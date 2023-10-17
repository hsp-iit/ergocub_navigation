#include "PathConverter/PathConverter_v2.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

PathConverter_v2::PathConverter_v2() : rclcpp::Node("path_converter_node")
{   
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

    m_shift_port.open(m_shift_portName);
    m_shiftFlag = false;
    
}

//Each time I have a new path we transform the path and pass it to the walking controller
void PathConverter_v2::msg_callback(const nav_msgs::msg::Path::ConstPtr& msg_in)
{
        std::lock_guard<std::mutex> lock(m_mutex);
        
        if (msg_counter < 4 && m_shiftFlag)
        {
            ++msg_counter;
            std::cout << "Msg counter: " << msg_counter << std::endl;
            return;
        }

        if(yarp::os::Network::isConnected("/dummy:o", "/path_converter/shift_command:i")){
            //Read from port
            auto data = m_shift_port.read(false);
            if (data!= nullptr)
            {
                std::cout << data->data()[0] << " " << data->data()[1] << " " << data->data()[2] << " " << std::endl;
                if ((data->data()[0])==1)
                {
                    std::cout << "Red from port" << std::endl;
                    m_shiftFlag = true;
                    m_shiftLeft = data->data()[1]==1 ? true : false;
                    msg_counter = 0;
                }
                else
                {
                    m_shiftFlag = false;
                }
            }
            //else    //JUST FOR DEBUG
            //{
            //    return; //TODO REMOVE
            //}
            
        } else {
            RCLCPP_ERROR(this->get_logger(), "Could not connect ports");
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
        std::cerr << "Received plan with zero length" << std::endl;
        throw std::runtime_error("Received plan with zero length");
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
    
    //
    if (m_shiftFlag)
    {
        std::cout << "Shifting Plan" << std::endl;
        //std::shared_ptr<nav_msgs::msg::Path> p =  std::make_shared<nav_msgs::msg::Path>(transformed_plan_);
        transformed_plan_ = shiftPlan(transformed_plan_, m_shiftLeft);
    }
    
    
        
    if (transformed_plan_.poses.empty()) {
        std::cerr << "Resulting plan has 0 poses in it." << std::endl;
        throw std::runtime_error("Resulting plan has 0 poses in it");
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
                /* code */
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
        auto node = std::make_shared<PathConverter_v2>();
        rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}