/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "HumanPoseGoalGenerator/CommunicationWrapper.hpp"

int main(int argc, char** argv)
{
    std::string humanPoseLocalName = "/HumanPoseGoalGenerator/pose:i";
    std::string humanPoseRemoteName = "/todo:o";
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;
    // Start listening in polling
    if (rclcpp::ok())
    {
        CommunicationWrapper dataProcessor;
        yarp::os::Port humanPoseGoalPort;
        humanPoseGoalPort.open(humanPoseLocalName);
        if(!yarp::os::Network::connect(humanPoseRemoteName, humanPoseLocalName))
        {
            yWarning() << "[HumanPoseGoalGenerator] unable to connect port: " << humanPoseRemoteName << " with " << humanPoseLocalName;
        }
        humanPoseGoalPort.setReader(dataProcessor);
        dataProcessor.spinNode();
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}
