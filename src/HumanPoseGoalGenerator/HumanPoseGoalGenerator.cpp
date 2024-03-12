/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "HumanPoseGoalGenerator/HumanPoseGoalGenerator.hpp"

HumanPoseGoalGenerator::HumanPoseGoalGenerator(){};

bool HumanPoseGoalGenerator::read(yarp::os::ConnectionReader& t_connection)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!m_initialized)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    //std::cout << "New message received" << std::endl;
    yarp::os::Bottle b;
    bool ok = b.read(t_connection);
    if (!ok) {
        std::cout << "No connection available for reading data " << std::endl;
        return false;
    }
    if (m_initialized)
    {
        try
        {
            m_rosNode->publishMarkers(b);
            //m_rosNode->publishGoal(b);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    else
    {
        std::cout << "ROS node not yet initialized" << std::endl;
    }
    return true;
}

void HumanPoseGoalGenerator::runROS()
{
    m_rosNode = std::make_shared<NodeRos>();
    m_initialized = true;
    rclcpp::spin(m_rosNode);
}