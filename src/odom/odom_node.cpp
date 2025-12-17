/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "odom/odom_node.hpp"
#include "yarp/os/Stamp.h"

using std::placeholders::_1;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

OdomNode::OdomNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("virtual_unicycle_publisher_node", options)
{
    declare_parameter("in_port_name", "/virtual_unicycle_publisher/unicycle_states:i");
    declare_parameter("out_port_name", "/navigation_helper/virtual_unicycle_states:o");
    declare_parameter("odom_topic_name", "/odom");
    declare_parameter("vel_topic", "/planned_vel");
    declare_parameter("loop_freq", 100.0);
    declare_parameter("nominal_width", 0.2);
    declare_parameter("odom_frame_name", "odom");
    declare_parameter("delta_x", 0.1);
    declare_parameter("ekf_enabled", false);

    declare_parameter("expose_ulterior_frames", false);

    // create TF
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
}

CallbackReturn OdomNode::on_configure(const rclcpp_lifecycle::State &)
{
    // Param Init
    m_in_port_name = this->get_parameter("in_port_name").as_string();
    m_out_port_name = this->get_parameter("out_port_name").as_string();
    m_odom_topic_name = this->get_parameter("odom_topic_name").as_string();
    m_vel_topic = this->get_parameter("vel_topic").as_string();
    m_loopFreq = this->get_parameter("loop_freq").as_double();
    m_nominalWidth = this->get_parameter("nominal_width").as_double();
    m_odom_frame_name = this->get_parameter("odom_frame_name").as_string();
    m_delta_x = this->get_parameter("delta_x").as_double();
    m_ekf_enabled = this->get_parameter("ekf_enabled").as_bool();
    m_expose_ulterior_frames = this->get_parameter("expose_ulterior_frames").as_bool();

    RCLCPP_INFO(get_logger(), "Configuring with: in_port_name: %s out_port_name: %s odom_topic_name: %s vel_topic: %s odom_frame_name: %s",
                    m_in_port_name.c_str(), m_out_port_name.c_str(), m_odom_topic_name.c_str(), m_vel_topic.c_str(), m_odom_frame_name.c_str());
    RCLCPP_INFO(get_logger(), "loop_freq: %f nominal_width: %f delta_x: %f ekf_enabled: %i expose_ulterior_frames: %i",
                    m_loopFreq, m_nominalWidth, m_delta_x, (int)m_ekf_enabled, (int)m_expose_ulterior_frames);

    // YARP Ports
    port.open(m_in_port_name);
    if (!yarp::os::Network::connect(m_out_port_name, m_in_port_name))
    {
        RCLCPP_INFO(get_logger(), "Failed to connect YARP ports");
        return CallbackReturn::FAILURE;
    }

    // Publishers
    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(m_odom_topic_name, 10);
    m_control_pub = this->create_publisher<geometry_msgs::msg::Twist>(m_vel_topic, 10);

    // Timer for looping thread
    auto duration = std::chrono::duration<double>(1 / m_loopFreq);
    m_timer = this->create_wall_timer(duration, std::bind(&OdomNode::PublishOdom, this));

    return CallbackReturn::SUCCESS;
}

CallbackReturn OdomNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");

    m_odom_pub->on_activate();
    m_control_pub->on_activate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn OdomNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");

    m_odom_pub->on_deactivate();
    m_control_pub->on_deactivate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn OdomNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning Up");
    m_odom_pub.reset();
    m_control_pub.reset();
    port.close();

    return CallbackReturn::SUCCESS;
}

CallbackReturn OdomNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());

    return CallbackReturn::SUCCESS;
}

CallbackReturn OdomNode::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

    return CallbackReturn::SUCCESS;
}

void OdomNode::PublishOdom()
{
    try
    {
        if (!(m_odom_pub->is_activated()))
        {
            RCLCPP_INFO(get_logger(), "Exiting PublishOdom: ublisher not active");
            return;
        }

        yarp::os::Bottle *data = port.read(true);
        yarp::os::Stamp stamp;
        port.getEnvelope(stamp);
        double time = stamp.getTime();
        std::vector<geometry_msgs::msg::TransformStamped> tfBuffer;

        // Optional extensive stuff
        if (m_expose_ulterior_frames)
        {
            // virtual unicycle by the walking-controller odometry (where the walking-controller thinks to be)
            geometry_msgs::msg::TransformStamped tf_fromOdom;
            tf_fromOdom.header.stamp.sec = (long int)time;
            tf_fromOdom.header.stamp.nanosec = (time - (long int)time) * 1E9;
            tf_fromOdom.header.frame_id = m_odom_frame_name;
            tf_fromOdom.child_frame_id = "odom_virtual_unicycle_simulated";
            tf_fromOdom.transform.translation.x = data->get(0).asList()->get(0).asFloat64();
            tf_fromOdom.transform.translation.y = data->get(0).asList()->get(1).asFloat64();
            tf_fromOdom.transform.translation.z = 0.0;

            tf2::Quaternion qVirtualUnicycleInOdomFrame;
            qVirtualUnicycleInOdomFrame.setRPY(0, 0, data->get(0).asList()->get(2).asFloat64());
            tf_fromOdom.transform.rotation.x = qVirtualUnicycleInOdomFrame.x();
            tf_fromOdom.transform.rotation.y = qVirtualUnicycleInOdomFrame.y();
            tf_fromOdom.transform.rotation.z = qVirtualUnicycleInOdomFrame.z();
            tf_fromOdom.transform.rotation.w = qVirtualUnicycleInOdomFrame.w();

            tfBuffer.push_back(tf_fromOdom);

            // Calculate the frames of the believed posiotion of the virtual unicycle by the walking-controller
            geometry_msgs::msg::TransformStamped tf, tfReference;
            tf.header.stamp.sec = (long int)time;
            tf.header.stamp.nanosec = (time - (long int)time) * 1E9;
            tfReference.header.stamp = tf.header.stamp;
            tf.child_frame_id = "virtual_unicycle_simulated";
            tfReference.child_frame_id = "virtual_unicycle_reference"; // this is ofsetted on y by a fixed quantity and it's used by the controller for tracking

            tf.transform.translation.x = 0;
            tf.transform.translation.z = 0;

            if (data->get(2).asString() == "left")
            {
                tf.header.frame_id = "l_sole";
                tfReference.header.frame_id = "l_sole";
                tf.transform.translation.y = -m_nominalWidth / 2;
            }
            else
            {
                tf.header.frame_id = "r_sole";
                tfReference.header.frame_id = "r_sole";
                tf.transform.translation.y = m_nominalWidth / 2;
            }

            tfReference.transform.translation.x = tf.transform.translation.x + m_delta_x;
            tfReference.transform.translation.y = tf.transform.translation.y;
            tfReference.transform.translation.z = 0.0;

            // Orientation Computation to make them planar
            geometry_msgs::msg::TransformStamped footToRootTF;
            if (data->get(2).asString() == "left")
            {
                footToRootTF = m_tf_buffer_in->lookupTransform("root_link", "l_sole", rclcpp::Time(0));
            }
            else
            {
                footToRootTF = m_tf_buffer_in->lookupTransform("root_link", "r_sole", rclcpp::Time(0));
            }
            // Conversion for extracting only YAW
            tf2::Quaternion tfGround; // quat of the root_link to chest frame tf
            tf2::fromMsg(footToRootTF.transform.rotation, tfGround);
            // Conversion from quat to rpy -> possible computational errors due to matricies
            tf2::Matrix3x3 m(tfGround);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            tfReference.transform.rotation.x = tf.transform.rotation.x = q.x();
            tfReference.transform.rotation.y = tf.transform.rotation.y = q.y();
            tfReference.transform.rotation.z = tf.transform.rotation.z = q.z();
            tfReference.transform.rotation.w = tf.transform.rotation.w = q.w();

            tfBuffer.push_back(tf);
            tfBuffer.push_back(tfReference);

            // Computation of the position of the virtual unicycle computed from the walking-controller in the odom frame
            geometry_msgs::msg::TransformStamped tfReference_fromOdom;
            tfReference_fromOdom.header.stamp = tf.header.stamp;
            tfReference_fromOdom.child_frame_id = "odom_virtual_unicycle_reference";
            tfReference_fromOdom.header.frame_id = m_odom_frame_name;

            // Virtual unicycle base pub in odom frame
            tfReference_fromOdom.transform.translation.x = data->get(1).asList()->get(0).asFloat64();
            tfReference_fromOdom.transform.translation.y = data->get(1).asList()->get(1).asFloat64();
            tfReference_fromOdom.transform.translation.z = 0.0;

            tfReference_fromOdom.transform.rotation = tf_fromOdom.transform.rotation;

            tfBuffer.push_back(tfReference_fromOdom);
        }

        // Odom Computation
        geometry_msgs::msg::TransformStamped odomTf;
        odomTf.header.frame_id = m_odom_frame_name;
        odomTf.child_frame_id = "root_link";
        odomTf.header.stamp.sec = (long int)time;
        odomTf.header.stamp.nanosec = (time - (long int)time) * 1E9;
        odomTf.transform.translation.x = data->get(3).asList()->get(0).asFloat64();
        odomTf.transform.translation.y = data->get(3).asList()->get(1).asFloat64();
        odomTf.transform.translation.z = data->get(3).asList()->get(2).asFloat64();

        tf2::Quaternion qOdom;
        qOdom.setRPY(data->get(3).asList()->get(3).asFloat64(), data->get(3).asList()->get(4).asFloat64(), data->get(3).asList()->get(5).asFloat64());
        odomTf.transform.rotation.x = qOdom.x();
        odomTf.transform.rotation.y = qOdom.y();
        odomTf.transform.rotation.z = qOdom.z();
        odomTf.transform.rotation.w = qOdom.w();
        if (!m_ekf_enabled)
        {
            tfBuffer.push_back(odomTf);
        }

        // odom msg publishing
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.frame_id = m_odom_frame_name;
        odom_msg.header.stamp = odomTf.header.stamp;
        odom_msg.pose.pose.position.x = odomTf.transform.translation.x;
        odom_msg.pose.pose.position.y = odomTf.transform.translation.y;
        odom_msg.pose.pose.position.z = odomTf.transform.translation.z;
        tf2::Quaternion tmp_quat;
        tmp_quat.setRPY(0, 0, data->get(3).asList()->get(5).asFloat64());
        // odom_msg.pose.pose.orientation=tf2::toMsg(tmp_quat);
        odom_msg.pose.pose.orientation = tf2::toMsg(qOdom);
        odom_msg.pose.covariance = m_pose_cov_matrix;

        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        // Publish measured vel of the CoM
        // geometry_msgs::msg::Twist measured_vel;
        try
        {
            odom_msg.twist.twist.linear.x = data->get(5).asList()->get(0).asFloat64();
            odom_msg.twist.twist.linear.y = data->get(5).asList()->get(1).asFloat64();
            // odom_msg.twist.twist.linear.z = 0.0;    //data->get(5).asList()->get(2).asFloat64();
            // odom_msg.twist.twist.angular.x = 0.0;   //data->get(5).asList()->get(3).asFloat64();
            // odom_msg.twist.twist.angular.y = 0.0;   //data->get(5).asList()->get(4).asFloat64();
            odom_msg.twist.twist.angular.z = data->get(5).asList()->get(5).asFloat64();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
        m_odom_pub->publish(odom_msg);

        // Should also publish the planned velocity for the root_link or CoM
        geometry_msgs::msg::Twist planned_vel;
        try
        {
            planned_vel.linear.x = data->get(4).asList()->get(0).asFloat64();
            planned_vel.linear.y = data->get(4).asList()->get(1).asFloat64();
            planned_vel.linear.z = 0.0;
            planned_vel.angular.x = 0.0;
            planned_vel.angular.y = 0.0;
            planned_vel.angular.z = 0.0; // data->get(4).asList()->get(2).asFloat64();
            m_control_pub->publish(planned_vel);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        // Geometrical virtual unicycle computed for reference
        geometry_msgs::msg::TransformStamped geometrycalVirtualUnicycle; // tf to broadcast
        geometry_msgs::msg::TransformStamped stanceFootToSwingFoot_tf;
        std::string stanceFoot, swingFoot;
        geometrycalVirtualUnicycle.header.stamp.sec = (long int)time;
        geometrycalVirtualUnicycle.header.stamp.nanosec = (time - (long int)time) * 1E9;
        geometrycalVirtualUnicycle.child_frame_id = "geometric_unicycle";
        // The unicycle pose and orientation will follow the swing foot ones ONLY if it has surpassed the stance foot (in the X direction)
        if (data->get(2).asString() == "left")
        {
            stanceFoot = "l_sole";
            swingFoot = "r_sole";
        }
        else
        {
            stanceFoot = "r_sole";
            swingFoot = "l_sole";
        }
        geometrycalVirtualUnicycle.header.frame_id = stanceFoot;
        stanceFootToSwingFoot_tf = m_tf_buffer_in->lookupTransform(swingFoot, stanceFoot, rclcpp::Time(0));

        // create a point in the swing foot frame center (0, 0, 0) and transform it in the stance foot frame
        // if X-component is negative, it means that it's behind it
        geometry_msgs::msg::PoseStamped swingFootCenter;
        swingFootCenter.header.frame_id = swingFoot;
        swingFootCenter.pose.position.x = .0;
        swingFootCenter.pose.position.y = .0;
        swingFootCenter.pose.position.z = .0;
        swingFootCenter.pose.orientation.x = .0;
        swingFootCenter.pose.orientation.y = .0;
        swingFootCenter.pose.orientation.z = .0;
        swingFootCenter.pose.orientation.w = 1;
        swingFootCenter = m_tf_buffer_in->transform(swingFootCenter, stanceFoot);
        // now let's check the relative position of the transformed point:
        if (swingFootCenter.pose.position.x > 0)
        {
            // if positive I take the swing foot as valid unicycle
            // let's take the yaw orientation of the swing foot
            tf2::Quaternion conversionQuat;
            tf2::fromMsg(stanceFootToSwingFoot_tf.transform.rotation, conversionQuat);
            tf2::Matrix3x3 matrix(conversionQuat);
            double r, p, y;
            matrix.getRPY(r, p, y);
            conversionQuat.setRPY(0.0, 0.0, -y);
            geometrycalVirtualUnicycle.transform.rotation.x = conversionQuat.x();
            geometrycalVirtualUnicycle.transform.rotation.y = conversionQuat.y();
            geometrycalVirtualUnicycle.transform.rotation.z = conversionQuat.z();
            geometrycalVirtualUnicycle.transform.rotation.w = conversionQuat.w();
            // translation -> take the halfway point on x and y
            geometrycalVirtualUnicycle.transform.translation.z = 0;
            geometrycalVirtualUnicycle.transform.translation.x = -stanceFootToSwingFoot_tf.transform.translation.x;
            geometrycalVirtualUnicycle.transform.translation.y = -stanceFootToSwingFoot_tf.transform.translation.y / 2;
        }
        else
        {
            // otherwise I keep the unicycle on the stance foot
            if (stanceFoot == "l_sole")
            {
                geometrycalVirtualUnicycle.transform.translation.y = -m_nominalWidth / 2; // depends by the nominalWidth/2 parameter in the walking-controller
            }
            else
            {
                geometrycalVirtualUnicycle.transform.translation.y = m_nominalWidth / 2;
            }
            geometrycalVirtualUnicycle.transform.translation.x = 0.0;
            geometrycalVirtualUnicycle.transform.translation.z = 0.0;
            geometrycalVirtualUnicycle.transform.rotation.x = 0;
            geometrycalVirtualUnicycle.transform.rotation.y = 0;
            geometrycalVirtualUnicycle.transform.rotation.z = 0;
            geometrycalVirtualUnicycle.transform.rotation.w = 1;
        }

        tfBuffer.push_back(geometrycalVirtualUnicycle);
        m_tf_broadcaster->sendTransform(tfBuffer);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

int main(int argc, char **argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);
    // Init YARP
    yarp::os::Network yarp;
    // Start listening in polling
    if (rclcpp::ok())
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        rclcpp::NodeOptions options;
        std::shared_ptr<OdomNode> node = std::make_shared<OdomNode>(options);
        executor.add_node(node->get_node_base_interface());
        executor.spin();
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}
