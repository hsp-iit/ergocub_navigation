/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "HumanPoseGoalGenerator/HumanPoseGoalGenerator.hpp"

int main(int argc, char** argv)
{
    const std::string humanPosePortName = "/todo";
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;
    // Start listening in polling
    if (rclcpp::ok())
    {
        HumanPoseGoalGenerator dataProcessor;
        yarp::os::Port humanPoseGoalPort;
        humanPoseGoalPort.open(humanPosePortName);
        // yarp::os::Network::connect("/navigation_helper/feet_positions:o", humanPosePortName);
        humanPoseGoalPort.setReader(dataProcessor);
        dataProcessor.runROS();
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}
