/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "StopActionNode/CommunicationWrapper.hpp"

int main(int argc, char** argv)
{
    std::string stopPortLocalName = "/StopActionNode/reader:i";
    std::string stopPortRemoteName = "/BT/stop_action/command:o";
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;
    // Start listening in polling
    if (rclcpp::ok())
    {
        CommunicationWrapper dataProcessor;
        yarp::os::Port stopActionPort;
        stopActionPort.open(stopPortLocalName);
        if(!yarp::os::Network::connect(stopPortRemoteName, stopPortLocalName))
        {
            yWarning() << "[StopActionNode] unable to connect port: " << stopPortRemoteName << " with " << stopPortLocalName;
        }
        stopActionPort.setReader(dataProcessor);
        dataProcessor.spinNode();
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}
