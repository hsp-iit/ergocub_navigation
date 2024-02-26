/**
 * @file    MotorControl.h
 * @author  Jon Woolfrey
 * @date    September 2023
 * @brief   A class for communicating with the joint motors on the ergoCub and iCub robots
 * @see     https://yarp.it/git-master/group__dev__iface__motor.html
 */

/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MOTORCONTROL_HPP_
#define MOTORCONTROL_HPP_

#include <Eigen/Core>                                                                               // Eigen::VectorXd
#include <iostream>                                                                                 // std::cerr, std::cout
#include <math.h>                                                                                   // M_PI
#include <string>                                                                                   // std::string
#include <vector>                                                                                   // std::vector
#include <yarp/dev/ControlBoardInterfaces.h>                                                        // I don't know what this does exactly...
#include <yarp/dev/PolyDriver.h>                                                                    // ... or this...
#include <yarp/os/Property.h>                                                                       // ... or this.

class MotorControl
{
	public:
		/**
		 * Default Constructor
		 */
		MotorControl()=default;
		
		/**
		 * Initializer
		 * @param jointList The list of joints on the robot to connect to, in order
		 * @param portList The names of the YARP ports for each of the joints
		 */
		bool init(const std::vector<std::string> &jointList,
		             const std::vector<std::string> &portList);
		
		/**
		 * Get the position and velocity values from the joint encoders.
		 * @param pos The joint positions
		 * @param vel The joint velocities
		 * @return Returns true if encoders were read successfully.
		 */
		bool read_encoders(std::vector<double> &pos, std::vector<double> &vel);
		
		/**
		 * Send reference positions to the motor controllers.
		 * @param references A list of desired joint positions
		 * @return Returns true if successful
		 */
		bool send_joint_commands(const std::vector<double> &commands);
		
		/**
		 * Closes the connection to the joint motors.
		 */
		void close();
		
        protected:
     
		unsigned int numJoints;                                                             ///< Number of joints being controlled
		
		std::vector<std::array<double,2>> positionLimit;                                    ///< Upper and lower bounds on joint position
		
		std::vector<double> velocityLimit;                                                  ///< Absolute joint velocity
		
		yarp::dev::IControlMode* mode;                                                      ///< Sets the control mode of the motor
	private:
		
	   	// These interface with the hardware on the robot itself
		yarp::dev::IControlLimits*   limits;                                                ///< Joint limits?
		yarp::dev::IEncoders*        encoders;                                              ///< Joint position values (in degrees)
		yarp::dev::IPositionDirect*  controller;                                            ///< As it says
		yarp::dev::PolyDriver        driver;                                                ///< Device driver
		bool m_initialized = false;
};                                                                                          

#endif