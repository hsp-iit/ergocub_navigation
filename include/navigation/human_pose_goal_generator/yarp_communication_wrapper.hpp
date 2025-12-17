/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef HUMAN_POSE_GOAL_GENERATOR__HPP
#define HUMAN_POSE_GOAL_GENERATOR__HPP

#include "yarp/os/BufferedPort.h"
#include "yarp/os/Network.h"
#include "yarp/sig/Vector.h"
#include "yarp/os/ConnectionWriter.h"
#include "yarp/os/Portable.h"
#include "yarp/os/LogStream.h"

#include "navigation/human_pose_goal_generator/goal_generator.hpp"
#include <thread>

class CommunicationWrapper : public yarp::os::PortReader
{
private:
    std::mutex m_mutex;
    std::shared_ptr<GoalGenerator> m_rosNode;
    bool m_initialized = false;

public:
    CommunicationWrapper();
    //main loop executed for each port reading of the merged feet status
    bool read(yarp::os::ConnectionReader& t_connection) override;

    void spinNode();
};  //End of class CommunicationWrapper : public yarp::os::PortReader

#endif
