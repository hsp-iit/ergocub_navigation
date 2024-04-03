/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FootstepsViewer/FootstepsViewerYarp.hpp"

int main(int argc, char** argv)
{
    const std::string footprintsPortName = "/footsteps_viewer/footsteps:i";
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;
    // Start listening in polling
    if (rclcpp::ok())
    {
        FootstepsViewerYarp dataProcessor;
        yarp::os::Port footprintsPort;
        footprintsPort.open(footprintsPortName);
        yarp::os::Network::connect("/navigation_helper/feet_positions:o", footprintsPortName);
        footprintsPort.setReader(dataProcessor);
        dataProcessor.runROS();
        //rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}
