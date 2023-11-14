#include <PhaseDetector/PhaseDetector.hpp>

using std::placeholders::_1;

PhaseDetector::PhaseDetector() : Node("phase_detector_node"), m_tfListener(nullptr)
{
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
    m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

    //Initialize Feet State in contact
    m_rightFootState=inContact;
    m_leftFootState=inContact;

    m_counter_rightSteps = 0;
    m_counter_leftSteps = 0;
    m_last_impact_time_left = std::chrono::high_resolution_clock::now();
    m_approaching_time_left = std::chrono::high_resolution_clock::now();
    m_last_impact_time_right = std::chrono::high_resolution_clock::now();
    m_approaching_time_right = std::chrono::high_resolution_clock::now();

    //Neck Controller
    m_joint_state = 0.0;
    m_startup = true;
    m_port.open(m_out_port_name);
    yarp::os::Network::connect(m_out_port_name, m_in_port_name);
    if(yarp::os::Network::isConnected(m_out_port_name, m_in_port_name)){
        RCLCPP_INFO(this->get_logger(), "YARP Ports connected successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Could not connect ports");
    }

    m_debug_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/swing_foot_height_debug", 10);
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
        std::thread worker(&PhaseDetector::gazeCallback, this, false);
        worker.join();
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
            if (tf.transform.translation.z >= m_tf_height_threshold_m)
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
            if (tf.transform.translation.z <= m_tf_height_threshold_m)
            {
                m_rightFootState = approachingContact;
                //TODO benchmarking time
                RCLCPP_INFO(this->get_logger(), "Right Foot aproaching contact");

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
        std::thread worker(&PhaseDetector::gazeCallback, this, true);
        worker.join();
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
        std::thread worker(&PhaseDetector::gazeCallback, this, true);
        worker.join();
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
            if (tf.transform.translation.z >= m_tf_height_threshold_m)
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
            if (tf.transform.translation.z <= m_tf_height_threshold_m)
            {
                m_leftFootState = approachingContact;
                RCLCPP_INFO(this->get_logger(), "Left Foot aproaching contact");

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

bool PhaseDetector::gazePattern(bool directionLeft)
{
    RCLCPP_INFO(this->get_logger(), "Entering gazePattern");
    double period = std::ceil(m_joint_limit_deg / m_joint_increment) * 2; //how many iterations I do from going to zero to the extreme, and back
    
    if (directionLeft)  //Looking Left
    {
        for (int i = 1; i <= period; ++i)
        {
            double setpoint = std::abs((double(i*2)/period) - 2.0*(std::trunc((double(i*2)/period) - std::trunc(double(i)/period)))) * m_joint_limit_deg;
            std::cout << "[gazePattern] LEFT Setting yaw setpoint of " << setpoint << std::endl;
            
            auto &data = m_port.prepare();
            data.clear();
            data = {0.0, 0.0, setpoint, 0.0};
            m_port.write();

            this->get_clock()->sleep_for(m_time_increment);
        }
    }
    else    //Looking Right
    {
        for (int i = 1; i <= period; ++i)
        {
            double setpoint = - std::abs((double(i*2)/period) - 2.0*(std::trunc((double(i*2)/period) - std::trunc(double(i)/period)))) * m_joint_limit_deg;
            std::cout << "[gazePattern] RIGHT Setting yaw setpoint of " << setpoint << std::endl;

            auto &data = m_port.prepare();
            data.clear();
            data = {0.0, 0.0, setpoint, 0.0};
            m_port.write();

            this->get_clock()->sleep_for(m_time_increment);
        }
    }
    
    return true;
}

void PhaseDetector::gazeCallback(bool directionLeft)
{
    RCLCPP_DEBUG(this->get_logger(), "Entering gazeCallback");
    //Scripred initial scan: Should be done in a BT node (as this whole function)
    if (m_startup)
    {
        RCLCPP_INFO(this->get_logger(), "Initial Scanning Around Routing");
        //this->get_clock()->sleep_for(3000ms);
        gazePattern(true);
        gazePattern(false);
        m_startup = false;
        return;
    }

    if (directionLeft)
    {
        gazePattern(true);
    }
    else //Look Right
    {
        gazePattern(false);
    }            
}

int main(int argc, char** argv)
{
    yarp::os::Network::init();
    rclcpp::init(argc, argv);
    if (rclcpp::ok())
    {
        auto node = std::make_shared<PhaseDetector>();
        std::cout << "Starting up node " << node->get_name() << " \n";
        rclcpp::spin(node);
        std::cout << "Shutting down" << std::endl;
        rclcpp::shutdown();
    }
    else
    {
        std::cout << "ROS2 not available. Shutting down node. \n";
    }
    return 0;
}