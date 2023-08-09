#include "BaseProjector/BaseProjector.hpp"

#include <mutex>
using namespace std::chrono_literals;

BaseProjector::BaseProjector() : rclcpp::Node("chest_projection_node")
{
    m_wrench_reader_port.open(m_reader_port_name);
    yarp::os::Network::connect(m_writer_port_name, m_reader_port_name);
    if(yarp::os::Network::isConnected(m_writer_port_name, m_reader_port_name))
    {
        RCLCPP_INFO(this->get_logger(), "YARP Ports connected successfully");
        m_ok = true;
    } 
    else 
    {
        RCLCPP_ERROR(this->get_logger(), "[YARP] /feetWrenches NOT PRESENT:\n execute the merge command on the feet wrenches:\n yarp merge --input /wholeBodyDynamics/right_foot_rear/cartesianEndEffectorWrench:o /wholeBodyDynamics/left_foot_rear/cartesianEndEffectorWrench:o --output /feetWrenches");
        m_ok = false;
    }

    m_foot_link = "r_sole";
    m_projection_TF.child_frame_id = "projection";
    m_projection_TF.header.frame_id = m_foot_link;
    /* init timer*/
    auto duration = std::chrono::duration<double>(1/m_loopFreq);
    m_timer_ = this->create_wall_timer( duration , std::bind(&BaseProjector::timer_callback, this));
    /* init TFs*/
    m_tf_pub = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
    m_virtual_unicycle_base_tf.child_frame_id = "virtual_unicycle_base";
    m_virtual_unicycle_base_tf.header.frame_id = m_foot_link;

    //Params declaration:
    if (!this->has_parameter("feet_sensor_threshold")) {
        this->declare_parameter((std::string)this->get_name() + ".feet_sensor_threshold", rclcpp::ParameterValue(100.0));
    }
    this->get_parameter((std::string)this->get_name() + ".feet_sensor_threshold", m_sensor_treshold);
    //debug
    RCLCPP_INFO(this->get_logger(), "Sensor threshold at: %f", m_sensor_treshold);
}

BaseProjector::~BaseProjector()
{
    try
    {
        m_wrench_reader_port.close();
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in closing yarp port: %s", e.what());
    }
}

void BaseProjector::timer_callback()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!m_ok)
    {
        RCLCPP_INFO(this->get_logger(), "FAILED TO CONNECT PORTS");
        return;
    }
    
    // Check which foot is on the ground based on the contact sensors
    yarp::os::Bottle* in_bottle = m_wrench_reader_port.read();
    //Determine which feet is in contact
    if (in_bottle->get(2).asFloat64() > m_sensor_treshold && in_bottle->get(8).asFloat64() < m_sensor_treshold)
    {
        m_foot_link = "l_sole";
        m_projection_TF.header.frame_id = m_foot_link;

        m_virtual_unicycle_base_tf.header.frame_id = m_foot_link;

        //RCLCPP_INFO(this->get_logger(), "Switching to: %s \n", foot_link);
    }
    else if (in_bottle->get(8).asFloat64() > m_sensor_treshold && in_bottle->get(2).asFloat64() < m_sensor_treshold)
    {
        m_foot_link = "r_sole";
        m_projection_TF.header.frame_id = m_foot_link;

        m_virtual_unicycle_base_tf.header.frame_id = m_foot_link;
        //RCLCPP_INFO(this->get_logger(), "Switching to: %s \n", foot_link);
    }
    else if(in_bottle->get(8).asFloat64() > m_sensor_treshold && in_bottle->get(2).asFloat64() > m_sensor_treshold)   //Both in contact
    {
        m_foot_link = "r_sole";
        m_projection_TF.header.frame_id = m_foot_link;
        m_virtual_unicycle_base_tf.header.frame_id = m_foot_link;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "No contacts on both the feets, values are left: %f and right: %f", in_bottle->get(2).asFloat64(), in_bottle->get(8).asFloat64());
        return;
    }

    // Get TF
    if (!get_TF(m_foot_link, m_chest_link))   
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot find TF between: %s & %s \n", m_chest_link, m_foot_link);
        return;
    }
    m_projection_TF.header.stamp = m_TF.header.stamp;   //timestamp is equal to the latest TF
    // Compute the projection to the same ground of the foot
    // get RPY
    tf2::Quaternion tf_quat;
    tf2::fromMsg(m_TF.transform.rotation, tf_quat);
    tf2::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    tf_quat.setRPY(0, 0, yaw);
    m_projection_TF.transform.translation.x = m_TF.transform.translation.x;
    m_projection_TF.transform.translation.y = m_TF.transform.translation.y;
    m_projection_TF.transform.translation.z = 0;
    m_projection_TF.transform.rotation = tf2::toMsg(tf_quat);
    //Computing of virtual unicycle base tf
    m_virtual_unicycle_base_tf.transform.translation.x = m_projection_TF.transform.translation.x;
    //The translational Y component is the mean between the feet sole distance
    if (m_foot_link=="r_sole")
    {
        if(!get_TF(m_foot_link, "l_sole"))
        {
            RCLCPP_ERROR(this->get_logger(), "[chest_projection] Cannot find TF between: %s & %s \n", m_foot_link, "l_sole");
            m_tf_pub->sendTransform(m_projection_TF);   //send atleast the available correct tf
            return;
        }
        m_virtual_unicycle_base_tf.transform.translation.y = m_TF.transform.translation.y/2;
    }
    else
    {
        if(!get_TF(m_foot_link, "r_sole"))
        {
            RCLCPP_ERROR(this->get_logger(), "[chest_projection] Cannot find TF between: %s & %s \n", m_foot_link, "r_sole");
            m_tf_pub->sendTransform(m_projection_TF);   //send atleast the available correct tf
            return;
        }
        m_virtual_unicycle_base_tf.transform.translation.y = m_TF.transform.translation.y/2;
    }
    m_virtual_unicycle_base_tf.header.stamp = m_TF.header.stamp;    //timestamp is equal to the latest TF
    m_virtual_unicycle_base_tf.transform.translation.z = 0;
    m_virtual_unicycle_base_tf.transform.rotation = m_projection_TF.transform.rotation;
    std::vector<geometry_msgs::msg::TransformStamped> tf_buffer;
    
    tf_buffer.push_back(m_projection_TF);
    tf_buffer.push_back(m_virtual_unicycle_base_tf);
    m_tf_pub->sendTransform(tf_buffer);
}

//get the transform for the data originated in the source_link to the frame expressed by the target_link
bool BaseProjector::get_TF(const std::string &target_link, const std::string &source_link)
{
    try
    {
        m_TF = m_tf_buffer_in->lookupTransform(target_link, source_link, rclcpp::Time(0), 200ms); // target link = chest
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s \n",ex.what());
        return false;
    }
    return true;
}

int main(int argc, char* argv[])
{
    // YARP init
    yarp::os::Network yarp;
    // ROS
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseProjector>();   
    if (rclcpp::ok()) 
    {
        std::cout << "Spinning chest_projection node" << std::endl;
        rclcpp::spin(node);
        std::cout << "Shutting down" << std::endl;
        rclcpp::shutdown();
    }
    return 0;
}