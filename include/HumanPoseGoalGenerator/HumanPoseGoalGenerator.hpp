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

#include "NodeRos.hpp"
#include <thread>

class HumanPoseGoalGenerator : public yarp::os::PortReader
{
private:
    std::mutex m_mutex;
    std::shared_ptr<NodeRos> m_rosNode;
    bool m_initialized = false;

public:
    HumanPoseGoalGenerator();
    //main loop executed for each port reading of the merged feet status
    bool read(yarp::os::ConnectionReader& t_connection) override;

    void runROS();
};  //End of class HumanPoseGoalGenerator : public yarp::os::PortReader

#endif
