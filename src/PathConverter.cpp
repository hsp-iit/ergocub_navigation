#include "PathConverter/PathConverter.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

PathConverter::PathConverter() : rclcpp::Node("path_converter_node")
{   
    m_setpoint_sub = this->create_subscription<nav_msgs::msg::Path>(
        m_topic_name,
        10,
        std::bind(&PathConverter::msg_callback, this, _1)
    );
    m_state_sub = this->create_subscription<std_msgs::msg::Bool>(
        m_state_topic,
        10,
        std::bind(&PathConverter::state_callback, this, _1)
    );

    // TFs
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
       
    //Create YarpFeetDataProcessor object
    m_port.open(m_outPortName);
    yarp::os::Network::connect(m_outPortName, m_inPortName);   
    if(yarp::os::Network::isConnected(m_outPortName, m_inPortName)){
        RCLCPP_INFO(this->get_logger(), "YARP Ports connected successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Could not connect ports");
    }
}

//Each time I have a new path we transform the path and pass it to the walking controller
void PathConverter::msg_callback(const nav_msgs::msg::Path::ConstPtr& msg_in)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        std::cout << "Original path size: " << msg_in->poses.size() << std::endl;
        if (!m_goalReached)
        {
            nav_msgs::msg::Path transformed_plan = *msg_in;
            if (msg_in->header.frame_id != m_reference_frame)
            {
                geometry_msgs::msg::TransformStamped TF = m_tf_buffer->lookupTransform(m_reference_frame, msg_in->header.frame_id, rclcpp::Time(0));
                transformed_plan = transformPlan(msg_in, TF, false);    //true
            }
            
            if (transformed_plan.poses.size()>0)
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
            else
            {
                std::cout << "Returned empty transformed path" << std::endl;
            }
        }
        else
        {
            //Write a path with all 0 poses
            auto& out = m_port.prepare();
            out.clear();
            for (int i = 0; i < 1; ++i)
            {
                out.push_back(0.0);
                out.push_back(0.0);
                out.push_back(0.0);
                std::cout << "Passing Path i-th element: " << i << " X : " << out[3*i] << " Y: " << out[3*i+1] << " Angle: " << out[3*i+2] << std::endl;
            }
            m_port.write();
        }
}

void PathConverter::state_callback(const std_msgs::msg::Bool::ConstPtr& in)
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
        }
        m_port.write();
    }
    else
    {
        m_goalReached = false;
    }
}

nav_msgs::msg::Path PathConverter::transformPlan(const nav_msgs::msg::Path::ConstPtr& path, 
                                                  geometry_msgs::msg::TransformStamped & t_tf, 
                                                  bool t_prune)
{
    if (path->poses.empty()) {
        std::cerr << "Received plan with zero length" << std::endl;
        throw std::runtime_error("Received plan with zero length");
    }

    // Transform the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan_ = *path;
    transformed_plan_.header.frame_id = "virtual_unicycle_base";   //virtual_unicycle_base
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
        
    if (transformed_plan_.poses.empty()) {
        std::cerr << "Resulting plan has 0 poses in it." << std::endl;
        throw std::runtime_error("Resulting plan has 0 poses in it");
    }
    return transformed_plan_;
}

int main(int argc, char** argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;

    // Start listening in polling
    if (rclcpp::ok())
    {
        auto node = std::make_shared<PathConverter>();
        rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}