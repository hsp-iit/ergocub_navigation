/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/trigger.hpp>

#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"
#include "yarp/os/BufferedPort.h"
#include "yarp/os/Network.h"
#include "yarp/sig/Vector.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

#include <memory>


std::atomic<bool> state = true;

void is_on_double_support(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] Received a request - state: %i", (int)state);
    response->success = state;
    state = false;  //reset the variable each time is called
}

//Class used for YARP port callbacks.
class YarpTriggerProcessor : public yarp::os::PortReader
{
private:
    std::mutex m_mutex;
    int m_footsteps_counter;
    const int m_step_number = 1;
public:
    YarpTriggerProcessor()
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] Created YarpTriggerProcessor");
        m_footsteps_counter = 0;
    };

    //main loop executed for each port reading of the merged feet status
    bool read(yarp::os::ConnectionReader& t_connection) override
    {
        try
        {
            //std::lock_guard<std::mutex> guard(m_mutex);
            yarp::os::Bottle b;
            bool ok = b.read(t_connection);
            if (!ok) {
                std::cout << "Bad Yarp connection " << std::endl;
                return false;
            }

        
            //state = b.get(0).asBool();
            //if (state)
            //{
            //    std::cout << "[is_on_double_support_srv] Red a trigger on YARP port" << std::endl;
            //}
            bool in_status = b.get(0).asBool();
            if (in_status)
            {
                std::cout << "[is_on_double_support_srv] Red a trigger on YARP port" << std::endl;
                ++ m_footsteps_counter;
                if (m_footsteps_counter >= m_step_number)
                {
                    state = true;
                    std::cout << "[is_on_double_support_srv] Setting Replanning" << std::endl;
                }
                else
                {
                    state = false;
                }    
            }
            else
            {
                state = false;
            }
            
            
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
        return true;
    }
};  //End of class YarpTriggerProcessor 

int main(int argc, char **argv)
{
    const std::string portName = "/planner_trigger_server/reader:i";
    const std::string sourceName = "/navigation_helper/replanning_trigger:o";
    rclcpp::init(argc, argv);
    yarp::os::Network yarp;
    yarp::os::Port port;

    yarp::os::BufferedPort<yarp::sig::Vector> walking_port;
    const std::string walking_sourceName = "/planner_trigger_server/walking_stop_path:o";
    const std::string walking_portName = "/walking-coordinator/goal:i";

    //Connect yarp ports
    walking_port.open(walking_sourceName);
    if (!yarp::os::Network::connect(walking_sourceName, walking_portName))
    {
        std::cout << "[planner_trigger_server - main] unable to connect: " << walking_sourceName << " with " << walking_portName << std::endl;
    }

    if (rclcpp::ok())
    {
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("is_on_double_support_srv");
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service = node->create_service<std_srvs::srv::Trigger>
                                                                     ("is_on_double_support_srv", &is_on_double_support);
        rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigation_result_sub = 
            node->create_subscription<action_msgs::msg::GoalStatusArray>(
                "navigate_to_pose/_action/status",
                rclcpp::SystemDefaultsQoS(),
                [&node, &walking_sourceName, &walking_portName, &walking_port](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {   
                    RCLCPP_INFO( node->get_logger(), "GOAL STATUS: %i", msg->status_list.back().status);
                    // Reset trigger status
                    bool stop = false;
                    switch (msg->status_list.back().status)
                    {
                    case 4:     // STATUS_SUCCEEDED 
                        state = true;
                        stop = true;    // TODO - maybe remove this
                        break;
                    case 5:     // STATUS_CANCELED  
                        state = true;
                        stop = true;
                        break;
                    case 6:     // STATUS_ABORTED   
                        state = true;
                        stop = true;
                        break;
                    default:    // Do nothing otherwise
                        break;
                    }

                    if (stop)
                    {
                        // try to stop the robot
                        try
                        {
                            if (!yarp::os::Network::isConnected(walking_sourceName, walking_portName))
                            {
                                yarp::os::Network::connect(walking_sourceName, walking_portName);
                            }
                            auto& out = walking_port.prepare();
                            out.clear();
                            for (int i = 0; i < 3; ++i)
                            {
                                out.push_back(0.0);
                                out.push_back(0.0);
                                out.push_back(0.0);
                                std::cout << "Passing Path i-th element: " << i << " X : " << out[3*i] << " Y: " << out[3*i+1] << " Angle: " << out[3*i+2] << std::endl;
                            }
                            walking_port.write();
                        }
                        catch(const std::exception& e)
                        {
                            std::cerr << e.what() << '\n';
                        }
                    }
                });
        //Connect yarp ports
        port.open(portName);
        yarp::os::Network::connect(sourceName, portName);
        YarpTriggerProcessor processor;

        if(yarp::os::Network::isConnected(sourceName, portName)){
            port.setReader(processor);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] YARP Ports connected successfully");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] Error connecting with YARP ports: %s with %s - Shutting Down", sourceName, portName);
            return 1;
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] Spinning service node");
        rclcpp::spin(node);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] Shutting down");
        rclcpp::shutdown();
    }
    
    
}
