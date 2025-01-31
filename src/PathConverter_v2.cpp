/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "PathConverter/PathConverter_v2.hpp"
#include <signal.h>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;

PathConverter_v2::PathConverter_v2(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("path_converter_node", options)
{   
    //Parameters Declaration
    declare_parameter("topic_name", "/plan");
    declare_parameter("state_topic", "/is_goal_reached");
    declare_parameter("outPortName", "/path_converter/path:o");
    declare_parameter("inPortName", "/walking-coordinator/goal:i");
    declare_parameter("reference_frame", "geometric_unicycle");
    declare_parameter("shift_portName", "/path_converter/shift_command:i");
    declare_parameter("shift_portConnectionName", "/shift_command:o");
    declare_parameter("zero_speed_threshold", 1e-03);
    declare_parameter("shift_enabled", true);
    declare_parameter("shift", 0.2);
    declare_parameter("max_msg_counter", 6);

    // TFs
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

CallbackReturn PathConverter_v2::on_configure(const rclcpp_lifecycle::State &)
{
    //Parameters reading
    m_topic_name = this->get_parameter("topic_name").as_string();
    m_state_topic = this->get_parameter("state_topic").as_string(); 
    m_outPortName = this->get_parameter("outPortName").as_string();
    m_inPortName = this->get_parameter("inPortName").as_string();
    m_reference_frame = this->get_parameter("reference_frame").as_string();
    m_shift_portName = this->get_parameter("shift_portName").as_string();
    m_shift_portConnectionName = this->get_parameter("shift_portConnectionName").as_string();
    m_zero_speed_threshold = this->get_parameter("zero_speed_threshold").as_double();
    m_shift_enabled = this->get_parameter("shift_enabled").as_bool();
    m_shift = this->get_parameter("shift").as_double();
    m_max_msg_counter = this->get_parameter("max_msg_counter").as_int();

    RCLCPP_INFO(get_logger(), "Configuring with: topic_name: %s state_topic: %s outPortName: %s inPortName: %s reference_frame: %s shift_portName: %s",
                    m_topic_name.c_str(), m_state_topic.c_str(), m_outPortName.c_str(), m_inPortName.c_str(), m_reference_frame.c_str(), m_shift_portName.c_str());
    RCLCPP_INFO(get_logger(), "shift_portConnectionName: %s zero_speed_threshold: %f shift_enabled: %i shift: %f max_msg_counter: %i",
                    m_shift_portConnectionName.c_str(), m_zero_speed_threshold, (int)m_shift_enabled, m_shift, m_max_msg_counter);

    //Subscribers
    m_setpoint_sub = this->create_subscription<nav_msgs::msg::Path>(
        m_topic_name,
        10,
        std::bind(&PathConverter_v2::msg_callback, this, _1)
    );
    m_state_sub = this->create_subscription<std_msgs::msg::Bool>(
        m_state_topic,
        10,
        std::bind(&PathConverter_v2::state_callback, this, _1)
    );

    //YARP port connection
    m_port.open(m_outPortName);
    yarp::os::Network::connect(m_outPortName, m_inPortName);   
    if(yarp::os::Network::isConnected(m_outPortName, m_inPortName)){
        RCLCPP_INFO(this->get_logger(), "YARP Ports connected successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Could not connect ports");
        return CallbackReturn::FAILURE;
    }

    if (m_shift_enabled)
    {
        m_shift_port.open(m_shift_portName);
        m_shiftFlag = false;
        //TODO add connection and its check TO ENABLE
        yarp::os::Network::connect(m_shift_portConnectionName, m_shift_portName);
        if (!yarp::os::Network::isConnected(m_shift_portConnectionName, m_shift_portName))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not connect ports for shift");
            //return CallbackReturn::FAILURE;
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Connected: " << m_shift_portConnectionName);
        }
        
    }
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn PathConverter_v2::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PathConverter_v2::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PathConverter_v2::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning Up");
    m_setpoint_sub.reset();
    m_state_sub.reset();
    if (!m_port.isClosed())
    {
        m_port.close();
    }
    if (m_shift_enabled && !m_shift_port.isClosed())
    {
        m_shift_port.close();
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn PathConverter_v2::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());
    try
        {
            RCLCPP_INFO(get_logger(), "Trying to stop the walking-controller");
            auto& out = m_port.prepare();
            out.clear();
            for (int i = 0; i < 3; ++i)
            {
                out.push_back(0.0);
                out.push_back(0.0);
                out.push_back(0.0);
                RCLCPP_INFO_STREAM(get_logger(), "Passing Path i-th element: " << i << " X : " << out[3*i] << " Y: " << out[3*i+1] << " Angle: " << out[3*i+2] );
            }
            m_port.write();
            RCLCPP_INFO(get_logger(), "Stop command sent");
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Got exception while passing stop command to walking-controller: " << e.what());
        }
    if (!m_port.isClosed())
    {
        m_port.close();
    }
    if (m_shift_enabled && !m_shift_port.isClosed())
    {
        m_shift_port.close();
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn PathConverter_v2::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

//Each time I have a new path we transform the path and pass it to the walking controller
void PathConverter_v2::msg_callback(const nav_msgs::msg::Path::ConstPtr& msg_in)
{
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_shift_enabled)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Shift Enabled" );
            if (msg_counter < m_max_msg_counter && m_shiftFlag)
            {
                ++msg_counter;
                RCLCPP_INFO_STREAM(get_logger(), "Msg counter: " << msg_counter);
                return;
            }

            //if(yarp::os::Network::isConnected(m_shift_portConnectionName, m_shift_portName)){
                //Read from port
                auto data = m_shift_port.read(false);
                if (data!= nullptr)
                {
                    //std::cout << data->data()[0] << " " << data->data()[1] << " " << data->data()[2] << " " << std::endl;
                    if (data->get(0).asInt32() == 1)   //data->data()[0]
                    {
                        std::cout << "Red from port" << std::endl;
                        m_shiftFlag = true;
                        //m_shiftLeft = data->data()[1]==1 ? true : false;
                        m_shiftLeft = (bool)data->get(1).asInt32();
                        std::cout << "m_shiftLeft: " << m_shiftLeft << std::endl;
                        msg_counter = 0;
                    }
                    else
                    {
                        m_shiftFlag = false;
                    }
                }
                else    //JUST FOR DEBUG
                {
                    //return; //TODO REMOVE
                    RCLCPP_WARN_STREAM(get_logger(), "Returned empty transformed path" );
                }

            //} else {
            //    RCLCPP_ERROR(this->get_logger(), "Could not connect ports");
            //}
        }

        std::cout << "Original path size: " << msg_in->poses.size() << std::endl;
        if (!m_goalReached)
        {
            nav_msgs::msg::Path transformed_plan = *msg_in;
            if (msg_in->header.frame_id != m_reference_frame)
            {
                geometry_msgs::msg::TransformStamped TF = m_tf_buffer->lookupTransform(m_reference_frame, msg_in->header.frame_id, rclcpp::Time(0));
                transformed_plan = transformPlan(msg_in, TF, false);    //true
            }
            
            if (transformed_plan.poses.size()>=3)
            {
                //Convert Path to yarp vector
                auto& out = m_port.prepare();
                out.clear();
                for (int i = 0; i < transformed_plan.poses.size(); ++i)
                {
                    out.push_back(transformed_plan.poses.at(i).pose.position.x);
                    out.push_back(transformed_plan.poses.at(i).pose.position.y);
                    //Angle conversion
                    tf2::Quaternion q;
                    tf2::fromMsg(transformed_plan.poses.at(i).pose.orientation, q);
                    tf2::Matrix3x3 conversion_matrix(q);
                    double roll, pitch, yaw;
                    conversion_matrix.getRPY(roll, pitch, yaw);
                    out.push_back(yaw);
                    std::cout << "Passing Path i-th element: " << i << " X : " << out[3*i] << " Y: " << out[3*i+1] << " Angle: " << out[3*i+2] << std::endl;
                }
                std::cout << "Original path size: " << transformed_plan.poses.size() << std::endl;
                //std::cout << "Writing port buffer" << std::endl;
                m_port.write();            
            }
            else if(transformed_plan.poses.size()>0 && transformed_plan.poses.size()<3)
            {
                //Convert Path to yarp vector
                auto& out = m_port.prepare();
                out.clear();
                // Fill the missing poses with zeroes
                for (int i = 0; i < 3 - transformed_plan.poses.size(); ++i)
                {
                    out.push_back(0.0);
                    out.push_back(0.0);
                    out.push_back(0.0);
                }
                int j = 1;
                for (int i = 0; i < transformed_plan.poses.size(); ++i)
                {
                    out.push_back(transformed_plan.poses.at(i).pose.position.x);
                    out.push_back(transformed_plan.poses.at(i).pose.position.y);
                    //Angle conversion
                    tf2::Quaternion q;
                    tf2::fromMsg(transformed_plan.poses.at(i).pose.orientation, q);
                    tf2::Matrix3x3 conversion_matrix(q);
                    double roll, pitch, yaw;
                    conversion_matrix.getRPY(roll, pitch, yaw);
                    out.push_back(yaw);
                    std::cout << "Passing Path i-th element: " << i << " X : " << out[3*(i + j)] << " Y: " << out[3*(i + j)+1] << " Angle: " << out[3*(i + j)+2] << std::endl;
                }
                std::cout << "Original path size: " << transformed_plan.poses.size() << std::endl;
                //std::cout << "Writing port buffer" << std::endl;
                m_port.write();   
            }
            else
            {
                RCLCPP_WARN_STREAM(get_logger(), "Returned empty transformed path" );
                // TODO - Should I stop the robot?
            }
        }
        else
        {
            //Write a path with all 0 poses
            auto& out = m_port.prepare();
            out.clear();
            for (int i = 0; i < 3; ++i)
            {
                out.push_back(0.0);
                out.push_back(0.0);
                out.push_back(0.0);
                RCLCPP_INFO_STREAM(get_logger(), "Passing Path i-th element: " << i << " X : " << out[3*i] << " Y: " << out[3*i+1] << " Angle: " << out[3*i+2] );
            }
            m_port.write();
        }
}

void PathConverter_v2::exit_handler(int signum) 
{
    ++ m_exit_count;
    
    // Send stop command only once
    if (m_exit_count == 1)
    {
        RCLCPP_INFO(this->get_logger(), "Caught CTRL+C, sending stop command to walking-controller");
        //Write a path with all 0 poses
        try
        {
            auto& out = m_port.prepare();
            out.clear();
            for (int i = 0; i < 3; ++i)
            {
                out.push_back(0.0);
                out.push_back(0.0);
                out.push_back(0.0);
                RCLCPP_INFO_STREAM(get_logger(), "Passing Path i-th element: " << i << " X : " << out[3*i] << " Y: " << out[3*i+1] << " Angle: " << out[3*i+2] );
            }
            m_port.write();
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Got exception while passing stop command to walking-controller: " << e.what());
        }
        // Terminate program
        exit(signum);
    }
    else if(m_exit_count > 5)
    {
        RCLCPP_INFO(this->get_logger(), "Caught too many CTRL+C, killing");
        exit(signum);
    }
    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Caught " << m_exit_count << " CTRL+C, ignoring");
    }
}

void PathConverter_v2::state_callback(const std_msgs::msg::Bool::ConstPtr& in)
{
    if (in->data)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_goalReached = true;
        //Write a path with all 0 poses
        auto& out = m_port.prepare();
        out.clear();
        for (int i = 0; i < 3; ++i)
        {
            out.push_back(0.0);
            out.push_back(0.0);
            out.push_back(0.0);
            RCLCPP_INFO_STREAM(get_logger(), "Passing Path i-th element: " << i << " X : " << out[3*i] << " Y: " << out[3*i+1] << " Angle: " << out[3*i+2] );
        }
        m_port.write();
    }
    else
    {
        m_goalReached = false;
    }
}

nav_msgs::msg::Path PathConverter_v2::transformPlan(const nav_msgs::msg::Path::ConstPtr& path, 
                                                  geometry_msgs::msg::TransformStamped & t_tf, 
                                                  bool t_prune)
{
    if (path->poses.empty()) {
        RCLCPP_ERROR_STREAM(get_logger(), "Received plan with zero length" );
        //throw std::runtime_error("Received plan with zero length");
    }

    // Transform the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan_ = *path;
    transformed_plan_.header.frame_id = m_reference_frame;   //virtual_unicycle_base
    transformed_plan_.header.stamp = path->header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't -> i.e the one with negative X
    // process it on the next iteration. (can't be done in odom frame)
    if (t_prune)
    {
        double min = 1e10;
        std::vector<geometry_msgs::msg::PoseStamped>::const_iterator index_found;
        geometry_msgs::msg::TransformStamped robot_path_pose = m_tf_buffer->lookupTransform("map","geometric_unicycle", rclcpp::Time(0));  //pose of the center of the feet on the path in the map frame geometric_unicycle
        std::cout << "robot_path_pose: X " << robot_path_pose.transform.translation.x << " Y: " << robot_path_pose.transform.translation.y << std::endl;
        for (auto it = transformed_plan_.poses.begin(); it != transformed_plan_.poses.end(); ++it)
        {
            double distance = sqrt(pow(robot_path_pose.transform.translation.x - it->pose.position.x, 2) + pow(robot_path_pose.transform.translation.y - it->pose.position.y, 2));  //distance of the center of the feet from each path pose
            std::cout << "Distance: " << distance << std::endl;
            if (distance < min)
            {
                min = distance;
                index_found = it;
                std::cout << "Found min at index: " << std::distance(transformed_plan_.poses.begin() , it) << std::endl;
            }
        }
        //erase the previous poses up to the point on the path where the robot is supposed to be closer
        if (index_found == transformed_plan_.poses.end() - 1)
        {
            transformed_plan_.poses.clear();
            return transformed_plan_;
        }
        
        transformed_plan_.poses.erase(transformed_plan_.poses.begin(), index_found + 1);
    }
    //Transform the (pruned) path
    //std::cout << "Transform the whole path for loop" << std::endl;
    for (int i = 0; i < transformed_plan_.poses.size(); ++i)
    {
        tf2::doTransform(transformed_plan_.poses.at(i), transformed_plan_.poses.at(i), t_tf);
        //std::cout << "Transformed X: " << transformed_plan_.poses.at(i).pose.position.x << "Transformed Y: " << transformed_plan_.poses.at(i).pose.position.y <<std::endl;
    }
    
    if (m_shift_enabled && m_shiftFlag)
    {
        std::cout << "Shifting Plan" << std::endl;
        //std::shared_ptr<nav_msgs::msg::Path> p =  std::make_shared<nav_msgs::msg::Path>(transformed_plan_);
        transformed_plan_ = shiftPlan(transformed_plan_, m_shiftLeft);
    }
        
    if (transformed_plan_.poses.empty()) {
        RCLCPP_ERROR_STREAM(get_logger(), "Resulting plan has 0 poses in it" );
    }
    return transformed_plan_;
}

nav_msgs::msg::Path PathConverter_v2::shiftPlan(const nav_msgs::msg::Path &path,
                                    bool directionLeft)
{
    nav_msgs::msg::Path path_out;
    path_out.header = path.header;
    path_out.poses.resize(path.poses.size() - 2);

    double delta; //how much I need to shift the path
    if (directionLeft)
        delta = m_shift;
    else
        delta =-m_shift;

    try
    {
        for (size_t i = 0; i < path.poses.size(); i++)
        {
            if (i == 0)
            {
                path_out.poses.at(i).pose = path.poses[i].pose;
            }
            else if (i < 3)
            {
               //do nothing -> keep the same poses
            }
            else
            {
                path_out.poses.at(i - 2).pose = path.poses[i].pose;
                path_out.poses.at(i - 2).pose.position.y += delta;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << "[shiftPlan] " << e.what() << '\n';
    }

    return  path_out.poses.size()>0 ? path_out : path;
}

int main(int argc, char** argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;

    if (rclcpp::ok())
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        rclcpp::NodeOptions options;
        std::shared_ptr<PathConverter_v2> node = std::make_shared<PathConverter_v2>(options);

        executor.add_node(node->get_node_base_interface());
        executor.spin();
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}