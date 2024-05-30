/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <PhaseDetector/PhaseDetector.hpp>

using std::placeholders::_1;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

PhaseDetector::PhaseDetector(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("phase_detector_node", options), 
                                m_tfListener(nullptr), 
                                m_rightFootState(inContact),
                                m_leftFootState(inContact)
{
    
    //Params Declaration
    declare_parameter("leftFoot_topic", "/left_foot_heel_tiptoe_ft");
    declare_parameter("rightFoot_topic", "/right_foot_heel_tiptoe_ft");
    declare_parameter("imu_topic", "/head_imu");
    declare_parameter("referenceFrame_right", "r_sole");
    declare_parameter("referenceFrame_left", "l_sole");
    declare_parameter("wrench_threshold", 80.0);
    declare_parameter("imu_threshold_y", 0.3);
    declare_parameter("tf_height_threshold", 0.02);
    declare_parameter("out_port_name", "/neck_controller/command:o");
    
    m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

    m_counter_rightSteps = 0;
    m_counter_leftSteps = 0;
    m_last_impact_time_left = std::chrono::high_resolution_clock::now();
    m_approaching_time_left = std::chrono::high_resolution_clock::now();
    m_last_impact_time_right = std::chrono::high_resolution_clock::now();
    m_approaching_time_right = std::chrono::high_resolution_clock::now();
    
    //Neck Controller
    m_startup = true;
}

CallbackReturn PhaseDetector::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Configuring node...");
    //Param Init
    m_leftFoot_topic = this->get_parameter("leftFoot_topic").as_string();
    m_rightFoot_topic = this->get_parameter("rightFoot_topic").as_string();
    m_imu_topic = this->get_parameter("imu_topic").as_string();
    m_referenceFrame_right = this->get_parameter("referenceFrame_right").as_string();
    m_referenceFrame_left = this->get_parameter("referenceFrame_left").as_string();
    m_wrench_threshold = this->get_parameter("wrench_threshold").as_double();
    m_imu_threshold_y = this->get_parameter("imu_threshold_y").as_double();
    m_tf_height_threshold = this->get_parameter("tf_height_threshold").as_double();
    m_out_port_name = this->get_parameter("out_port_name").as_string();

    RCLCPP_INFO(get_logger(), "Configuring with: leftFoot_topic: %s rightFoot_topic: %s imu_topic: %s reference_frame_right: %s reference_frame_left: %s out_port_name: %s",
                    m_leftFoot_topic.c_str(), m_rightFoot_topic.c_str(), m_imu_topic.c_str(), m_referenceFrame_right.c_str(), m_referenceFrame_left.c_str(), m_out_port_name.c_str());
    RCLCPP_INFO(get_logger(), "wrench_threshold: %f imu_threshold_y: %f tf_height_threshold: %f ", m_wrench_threshold, m_imu_threshold_y, m_tf_height_threshold);

    //Subscribers
    m_rightFoot_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                                                m_rightFoot_topic,
                                                10, 
                                                std::bind(&PhaseDetector::rightFootCallback, this, _1));
    m_leftFoot_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                                                m_leftFoot_topic,
                                                10,
                                                std::bind(&PhaseDetector::leftFootCallback, this, _1));
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                                                m_imu_topic,
                                                10,
                                                std::bind(&PhaseDetector::imuCallback, this, _1));
    //Pub
    m_debug_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/swing_foot_height_debug", 10);

    m_port.open(m_out_port_name);
    yarp::os::Network::connect(m_out_port_name, m_remote_port_name);
    if(yarp::os::Network::isConnected(m_out_port_name, m_remote_port_name)){
        RCLCPP_INFO(this->get_logger(), "YARP Ports connected successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Could not connect ports: %s with %s ", m_out_port_name, m_remote_port_name);
    }

    RCLCPP_INFO(this->get_logger(), "Node configured");
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhaseDetector::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Activating");
    m_debug_pub->on_activate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhaseDetector::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Deactivating");
    m_debug_pub->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhaseDetector::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning Up");
    m_rightFoot_sub.reset();
    m_imu_sub.reset();
    m_leftFoot_sub.reset();
    m_debug_pub.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhaseDetector::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());
  m_port.close();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PhaseDetector::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

    return CallbackReturn::SUCCESS;
}

PhaseDetector::~PhaseDetector()
{
}

void PhaseDetector::rightFootCallback(const geometry_msgs::msg::WrenchStamped::ConstPtr &msg)
{
    if (-msg->wrench.force.z >= m_wrench_threshold)  //detecting contact
    {
        if (m_rightFootState != inContact)
        {
            std::cout << "m_rightFootState != inContact ";
            ++m_counter_rightSteps;
            m_last_impact_time_right = std::chrono::high_resolution_clock::now();
            RCLCPP_INFO(this->get_logger(), "Right Foot stepping in contact Step n: %i", m_counter_rightSteps);
            auto d_t = std::chrono::duration_cast<std::chrono::milliseconds>(m_approaching_time_right - m_last_impact_time_right);
            RCLCPP_INFO(this->get_logger(), "Difference between previous approaching phase: %e ms", d_t);
        }
        m_rightFootState = inContact;
    }
    else if (m_rightFootState == inContact) //contact not detected but was in contact the msg before -> FOR DEBUG
    {
        RCLCPP_INFO(this->get_logger(), "Right Foot leaving contact");
        m_rightFootState = leavingContact;
        PhaseDetector::sendCommand(RIGHT);
    }
    else if (m_rightFootState == leavingContact)    //check height of the foot if reaching apex zone
    {
        try
        {
            auto tf = m_tfBuffer->lookupTransform(m_referenceFrame_left, m_referenceFrame_right, tf2::TimePointZero);
            std::cout << "RIGHT Z: " << tf.transform.translation.z << std::endl;
            geometry_msgs::msg::PointStamped data;
            data.header.frame_id = "r_sole";
            data.header.stamp = this->get_clock()->now();
            data.point.x = 0.0;
            data.point.y = 0.0;
            data.point.z = tf.transform.translation.z;
            m_debug_pub->publish(data);
            if (tf.transform.translation.z >= m_tf_height_threshold)
            {
                m_rightFootState = apexZone;
                RCLCPP_INFO(this->get_logger(), "Right Foot apex zone");
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    else if (m_rightFootState == apexZone)
    {
        try
        {
            auto tf = m_tfBuffer->lookupTransform(m_referenceFrame_left, m_referenceFrame_right, tf2::TimePointZero);
            geometry_msgs::msg::PointStamped data;
            data.header.frame_id = "r_sole";
            data.header.stamp = this->get_clock()->now();
            data.point.x = 0.0;
            data.point.y = 0.0;
            data.point.z = tf.transform.translation.z;
            m_debug_pub->publish(data);
            if (tf.transform.translation.z <= m_tf_height_threshold)
            {
                m_rightFootState = approachingContact;
                //TODO benchmarking time
                RCLCPP_INFO(this->get_logger(), "Right Foot approaching contact");

                m_approaching_time_right = std::chrono::high_resolution_clock::now();
                auto d_t = std::chrono::duration_cast<std::chrono::milliseconds>(m_last_impact_time_right - m_approaching_time_right);
                RCLCPP_INFO(this->get_logger(), "Duration from previous contact: %f", d_t);
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    //else nothing
}

void PhaseDetector::leftFootCallback(const geometry_msgs::msg::WrenchStamped::ConstPtr &msg)
{
    if (m_startup)
    {
        PhaseDetector::sendCommand(FULL_SWEEP);
        return;
    }
    
    if (-msg->wrench.force.z >= m_wrench_threshold)  //detecting contact
    {
        if (m_leftFootState != inContact)
        {
            ++m_counter_leftSteps;
            m_last_impact_time_left = std::chrono::high_resolution_clock::now();
            std::cout << "Left Foot stepping in contact Step n: " << m_counter_leftSteps << std::endl;
            //RCLCPP_DEBUG(this->get_logger(), "Left Foot stepping in contact Step n: %i", m_counter_leftSteps);
            auto d_t = std::chrono::duration_cast<std::chrono::milliseconds>(m_approaching_time_left - m_last_impact_time_left);
            RCLCPP_INFO(this->get_logger(), "[leftFootCallback] Difference between previous approaching phase: %e ms", d_t);
        }
        m_leftFootState = inContact;
    }
    else if (m_leftFootState == inContact) //contact not detected but was in contact the msg before -> FOR DEBUG
    {
        RCLCPP_INFO(this->get_logger(), "Left Foot leaving contact");
        m_leftFootState = leavingContact;
        PhaseDetector::sendCommand(LEFT);
    }
    else if (m_leftFootState == leavingContact)    //check height of the foot if reaching apex zone
    {
        try
        {
            auto tf = m_tfBuffer->lookupTransform(m_referenceFrame_right, m_referenceFrame_left, tf2::TimePointZero);
            std::cout << "LEFT Z: " << tf.transform.translation.z << std::endl;
            geometry_msgs::msg::PointStamped data;
            data.header.frame_id = "l_sole";
            data.header.stamp = this->get_clock()->now();
            data.point.x = 0.0;
            data.point.y = 0.0;
            data.point.z = tf.transform.translation.z;
            m_debug_pub->publish(data);
            if (tf.transform.translation.z >= m_tf_height_threshold)
            {
                m_leftFootState = apexZone;
                RCLCPP_INFO(this->get_logger(), "Left Foot apex zone");
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    else if (m_leftFootState == apexZone)
    {
        try
        {
            auto tf = m_tfBuffer->lookupTransform(m_referenceFrame_right, m_referenceFrame_left, tf2::TimePointZero);
            geometry_msgs::msg::PointStamped data;
            data.header.frame_id = "l_sole";
            data.header.stamp = this->get_clock()->now();
            data.point.x = 0.0;
            data.point.y = 0.0;
            data.point.z = tf.transform.translation.z;
            m_debug_pub->publish(data);
            if (tf.transform.translation.z <= m_tf_height_threshold)
            {
                m_leftFootState = approachingContact;
                RCLCPP_INFO(this->get_logger(), "Left Foot approaching contact");

                m_approaching_time_left = std::chrono::high_resolution_clock::now();
                auto d_t = std::chrono::duration_cast<std::chrono::milliseconds>(m_last_impact_time_left - m_approaching_time_left);
                RCLCPP_INFO(this->get_logger(), "Duration from previous contact: %e", d_t);
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    //else nothing
}

void PhaseDetector::imuCallback(const sensor_msgs::msg::Imu::ConstPtr &msg)
{
    if (msg->angular_velocity.y >= m_imu_threshold_y)
    {
        m_vibration_time = std::chrono::high_resolution_clock::now();
        std::chrono::milliseconds d_t;
        if (m_approaching_time_right > m_approaching_time_left) //swinging with right foot
        {
            d_t = std::chrono::duration_cast<std::chrono::milliseconds>(m_vibration_time - m_last_impact_time_right);
        }
        else    //left foot
        {
            d_t = std::chrono::duration_cast<std::chrono::milliseconds>(m_vibration_time - m_last_impact_time_left);
        }
        RCLCPP_INFO(this->get_logger(), "Vibration time from previous impact time: %f", d_t);
    }    
}

bool PhaseDetector::sendCommand(int command)
{
    // command: 0 go home, 1 left, 2 right, 3 full sweep
    try
    {
        auto& data = m_port.prepare();
        data.clear();
        data.addInt32(command);
        m_port.write();
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[PhaseDetector::sendCommand] Exception: " << e.what());
        return false;
    }
    return true;
}

int main(int argc, char** argv)
{
    yarp::os::Network::init();
    rclcpp::init(argc, argv);
    if (rclcpp::ok())
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        rclcpp::NodeOptions options;
        std::shared_ptr<PhaseDetector> node = std::make_shared<PhaseDetector>(options);

        executor.add_node(node->get_node_base_interface());
        executor.spin();
        std::cout << "Shutting down" << std::endl;
        rclcpp::shutdown();
    }
    else
    {
        std::cout << "ROS2 not available. Shutting down node. \n";
    }
    return 0;
}