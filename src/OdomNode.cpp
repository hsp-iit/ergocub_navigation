#include "OdomNode/OdomNode.hpp"
#include "yarp/os/Stamp.h"


OdomNode::OdomNode() : rclcpp::Node("virtual_unicycle_publisher_node")
{   
    port.open(port_name);
    yarp::os::Network::connect("/navigation_helper/virtual_unicycle_states:o", port_name); 
    //create TF
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(m_odom_topic_name, 10);
    m_control_pub = this->create_publisher<geometry_msgs::msg::Twist>("/planned_vel", 10);
    auto duration = std::chrono::duration<double>(1/m_loopFreq);
    m_timer = this->create_wall_timer(duration, std::bind(&OdomNode::PublishOdom, this));
}

void OdomNode::PublishOdom()
    {
        try
        {
            yarp::os::Bottle* data = port.read(true);
            yarp::os::Stamp stamp;
            port.getEnvelope(stamp);
            double time = stamp.getTime();
            //TODO use time stamp from YARP
            long long sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
            long long ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
            ns = ns%((long long)10e9);
            //RCLCPP_INFO(this->get_logger(), "from yarp TS: %f ; from local clock: %i s - %i ns", time, sec, ns);
            std::vector<geometry_msgs::msg::TransformStamped> tfBuffer;
            geometry_msgs::msg::TransformStamped tf, tfReference;
            tf.header.stamp.sec = (long int)time;
            tf.header.stamp.nanosec = (time - (long int)time) * 1E9;
            //RCLCPP_INFO(this->get_logger(), "from yarp TS: %f ; to STAMP: %i s - %i ns", time, tf.header.stamp.sec, tf.header.stamp.nanosec);
            tfReference.header.stamp = tf.header.stamp;
            geometry_msgs::msg::TransformStamped tf_fromOdom, tfReference_fromOdom; //position of the virtual unicycle computed from the walking-controller in the odom frame
            tf_fromOdom.header.stamp = tfReference_fromOdom.header.stamp = tf.header.stamp;
            tf_fromOdom.child_frame_id = "odom_virtual_unicycle_simulated";
            tfReference_fromOdom.child_frame_id = "odom_virtual_unicycle_reference";
            tf_fromOdom.header.frame_id = "odom";
            tfReference_fromOdom.header.frame_id = "odom";

            if (data->get(2).asString() == "left")
            {
                tf.header.frame_id = "l_sole";
                tfReference.header.frame_id = "l_sole";
                tf.child_frame_id = "virtual_unicycle_simulated";
                tfReference.child_frame_id = "virtual_unicycle_reference";
                tf.transform.translation.y = - m_nominalWidth/2;
            }
            else
            {
                tf.header.frame_id = "r_sole";
                tfReference.header.frame_id = "r_sole";
                tf.child_frame_id = "virtual_unicycle_simulated"; 
                tfReference.child_frame_id = "virtual_unicycle_reference";   
                tf.transform.translation.y = m_nominalWidth/2;        
            }

            tf.transform.translation.x = 0;
            tf.transform.translation.z = 0;
            tfReference.transform.translation.x = tf.transform.translation.x + 0.1;
            tfReference.transform.translation.y = tf.transform.translation.y;
            tfReference.transform.translation.z = 0.0;
            //Odom Computation
            geometry_msgs::msg::TransformStamped odomTf;
            odomTf.header.frame_id = "odom";
            odomTf.child_frame_id = "root_link";
            odomTf.header.stamp = tf.header.stamp;
            odomTf.transform.translation.x = data->get(3).asList()->get(0).asFloat64();
            odomTf.transform.translation.y = data->get(3).asList()->get(1).asFloat64();
            odomTf.transform.translation.z = data->get(3).asList()->get(2).asFloat64();

            tf2::Quaternion qOdom;
            qOdom.setRPY(data->get(3).asList()->get(3).asFloat64(), data->get(3).asList()->get(4).asFloat64(), data->get(3).asList()->get(5).asFloat64());
            odomTf.transform.rotation.x = qOdom.x();
            odomTf.transform.rotation.y = qOdom.y();
            odomTf.transform.rotation.z = qOdom.z();
            odomTf.transform.rotation.w = qOdom.w();
            tfBuffer.push_back(odomTf);

            //odom msg publishing
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.frame_id = "odom";
            odom_msg.header.stamp = odomTf.header.stamp;
            odom_msg.pose.pose.position.x = odomTf.transform.translation.x;
            odom_msg.pose.pose.position.y = odomTf.transform.translation.y;
            odom_msg.pose.pose.position.z = 0.0;
            tf2::Quaternion tmp_quat;
            tmp_quat.setRPY(0, 0, data->get(3).asList()->get(5).asFloat64());
            odom_msg.pose.pose.orientation=tf2::toMsg(tmp_quat);
            odom_msg.pose.covariance = m_pose_cov_matrix;

            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.linear.z = 0.0;
            odom_msg.twist.twist.angular.x = 0.0;
            odom_msg.twist.twist.angular.y = 0.0;
            odom_msg.twist.twist.angular.z = 0.0;

            //Publish measured vel of the CoM
            //geometry_msgs::msg::Twist measured_vel;
            try
            {
                odom_msg.twist.twist.linear.x = data->get(5).asList()->get(0).asFloat64();
                odom_msg.twist.twist.linear.y = data->get(5).asList()->get(1).asFloat64();
                //odom_msg.twist.twist.linear.z = 0.0;    //data->get(5).asList()->get(2).asFloat64();
                //odom_msg.twist.twist.angular.x = 0.0;   //data->get(5).asList()->get(3).asFloat64();
                //odom_msg.twist.twist.angular.y = 0.0;   //data->get(5).asList()->get(4).asFloat64();
                odom_msg.twist.twist.angular.z = data->get(5).asList()->get(5).asFloat64();
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            m_odom_pub->publish(odom_msg);

            //Should also publish the planned velocity for the root_link or CoM
            geometry_msgs::msg::Twist planned_vel;
            try
            {
                planned_vel.linear.x = data->get(4).asList()->get(0).asFloat64();
                planned_vel.linear.y = data->get(4).asList()->get(1).asFloat64();
                planned_vel.linear.z = 0.0;
                planned_vel.angular.x = 0.0;
                planned_vel.angular.y = 0.0;
                planned_vel.angular.z = 0.0; //data->get(4).asList()->get(2).asFloat64();
                m_control_pub->publish(planned_vel);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
            //Virtual unicycle base pub in odom frame
            tf_fromOdom.transform.translation.x = data->get(0).asList()->get(0).asFloat64();
            tf_fromOdom.transform.translation.y = data->get(0).asList()->get(1).asFloat64();
            tf_fromOdom.transform.translation.z = 0.0;

            tfReference_fromOdom.transform.translation.x = data->get(1).asList()->get(0).asFloat64();
            tfReference_fromOdom.transform.translation.y = data->get(1).asList()->get(1).asFloat64();
            tfReference_fromOdom.transform.translation.z = 0.0;

            tf2::Quaternion qVirtualUnicycleInOdomFrame;
            qVirtualUnicycleInOdomFrame.setRPY(0, 0, data->get(0).asList()->get(2).asFloat64());
            tf_fromOdom.transform.rotation.x = qVirtualUnicycleInOdomFrame.x();
            tf_fromOdom.transform.rotation.y = qVirtualUnicycleInOdomFrame.y();
            tf_fromOdom.transform.rotation.z = qVirtualUnicycleInOdomFrame.z();
            tf_fromOdom.transform.rotation.w = qVirtualUnicycleInOdomFrame.w();
            tfReference_fromOdom.transform.rotation = tf_fromOdom.transform.rotation;

            geometry_msgs::msg::TransformStamped footToRootTF;
            if (data->get(2).asString() == "left")
            {
                footToRootTF = m_tf_buffer_in->lookupTransform("root_link", "l_sole", rclcpp::Time(0));
            }
            else
            {
                footToRootTF = m_tf_buffer_in->lookupTransform("root_link", "r_sole", rclcpp::Time(0));
            }
            //Conversion for extracting only YAW
            tf2::Quaternion tfGround;    //quat of the root_link to chest frame tf
            tf2::fromMsg(footToRootTF.transform.rotation, tfGround);
            //Conversion from quat to rpy -> possible computational errors due to matricies
            tf2::Matrix3x3 m(tfGround);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();

            tf2::Quaternion qRef;
            qRef.setRPY(0, 0, yaw);
            tfReference.transform.rotation.x = qRef.x();
            tfReference.transform.rotation.y = qRef.y();
            tfReference.transform.rotation.z = qRef.z();
            tfReference.transform.rotation.w = qRef.w();

            tfBuffer.push_back(tf);
            tfBuffer.push_back(tfReference);
            tfBuffer.push_back(tf_fromOdom);
            tfBuffer.push_back(tfReference_fromOdom);

            //Geometrical virtual unicycle approach
            geometry_msgs::msg::TransformStamped geometrycalVirtualUnicycle;    //tf to broadcast
            geometry_msgs::msg::TransformStamped stanceFootToSwingFoot_tf;
            std::string stanceFoot, swingFoot;
            geometrycalVirtualUnicycle.header.stamp = tf.header.stamp;
            geometrycalVirtualUnicycle.child_frame_id = "geometric_unicycle";
            //The unicycle pose and orientation will follow the swing foot ones ONLY if it has surpassed the stance foot (in the X direction)
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
            
            //create a point in the swing foot frame center (0, 0, 0) and transform it in the stance foot frame
            //if X-component is negative, it means that it's behind it
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
            //now let's check the relative position of the transformed point:
            if (swingFootCenter.pose.position.x > 0)
            {
                //if positive I take the swing foot as valid unicycle
                //let's take the yaw orientation of the swing foot
                tf2::Quaternion conversionQuat;
                tf2::fromMsg(stanceFootToSwingFoot_tf.transform.rotation, conversionQuat);
                tf2::Matrix3x3 matrix(conversionQuat);
                double r, p, y;
                matrix.getRPY(r, p, y);
                conversionQuat.setRPY(0.0, 0.0, - y);
                geometrycalVirtualUnicycle.transform.rotation.x = conversionQuat.x();
                geometrycalVirtualUnicycle.transform.rotation.y = conversionQuat.y();
                geometrycalVirtualUnicycle.transform.rotation.z = conversionQuat.z();
                geometrycalVirtualUnicycle.transform.rotation.w = conversionQuat.w();
                //translation -> take the halfway point on x and y
                geometrycalVirtualUnicycle.transform.translation.z = 0;
                geometrycalVirtualUnicycle.transform.translation.x = -stanceFootToSwingFoot_tf.transform.translation.x;
                geometrycalVirtualUnicycle.transform.translation.y = -stanceFootToSwingFoot_tf.transform.translation.y / 2;
            }
            else
            {
                //otherwise I keep the unicycle on the stance foot
                if (stanceFoot == "l_sole")
                {
                    geometrycalVirtualUnicycle.transform.translation.y = -m_nominalWidth/2; //depends by the nominalWidth/2 parameter in the walking-controller
                }
                else
                {
                    geometrycalVirtualUnicycle.transform.translation.y = m_nominalWidth/2;
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
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
        }
    }

int main(int argc, char** argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;
    const std::string port_name = "/virtual_unicycle_publisher/unicycle_state:i";
    // Start listening in polling
    if (rclcpp::ok())
    {
        auto node = std::make_shared<OdomNode>();
        rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}
