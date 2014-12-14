/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2013 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: ipa_canopen
 * \note
 *   ROS stack name: ipa_canopen
 * \note
 *   ROS package name: ipa_canopen_ros
 *
 * \author
 *   Author: Thiago de Freitas, Tobias Sing, Eduard Herkel
 * \author
 *   Supervised by: Thiago de Freitas email:tdf@ipa.fhg.de
 *
 * \date Date of creation: December 2012
 *
 * \brief
 *   Implementation of canopen.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include "ros/ros.h"
#include <urdf/model.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <iostream>
#include <map>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <ipa_canopen_core/canopen.h>
#include <XmlRpcValue.h>
#include <ipa_canopen_ros/JointLimits.h>
#include <iostream>


int msleep(int ms) {
  usleep(ms*1000);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ipa_canopen_ros");
    ros::NodeHandle n("");

    if (argc != 2) {
      std::cerr << "usage: " << argv[0] << " nodeId " <<  std::endl;
      exit(-1);
    }

//     ros::Publisher jointStatesPublisher = n.subscribe<geometry_msgs::Twist>("/skid_sterring", 10, );

    double lr = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(canopen::syncInterval).count();

    canopen::openConnection("", "");
    msleep(100);

    int canId = atoi(argv[1]);
    std::cout << " starting node " << canId << std::endl;

    canopen::sendNMT(0x00, NMT_RESET_COMMUNICATION);
    msleep(5000);

    canopen::sendSDO(canId, canopen::CONTROLWORD, canopen::CONTROLWORD_ENABLE_OPERATION);
    msleep(5000);
    canopen::receiveCanPacket();
    canopen::receiveCanPacket();
    canopen::receiveCanPacket();

//     canopen::sendSDO(canId, canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_VELOCITY_MODE);
    canopen::sendSDO(canId, canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_PROFILE_VELOCITY_MODE);
    msleep(300);
    canopen::receiveCanPacket();
    msleep(300);

    canopen::sendSDO(canId, canopen::TARGET_VELOCITY, 20000);
    canopen::receiveCanPacket();
    msleep(300);

    canopen::sendSDO(canId, canopen::CONTROLWORD, canopen::CONTROLWORD_ENABLE_MOVEMENT);
    msleep(500);
    canopen::receiveCanPacket();


    ros::Rate loop_rate(lr);

    return 0;
}

