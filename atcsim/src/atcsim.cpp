/*
 * atcsim.cpp
 *
 *  Created on: 07/2020
 *
 *
 *  Copyright 2020 Daniel Garcia
 *
 *  This file is part of ATCSimROS.
 *
 *  ATCSimROS is free software based on the ATCsim project , created for a
 *	final project of the Aerospace Engineering at URJC (Madrid).
 *	Author: Daniel Garcia Herraiz (danielghgr@gmail.com)
 *	Tutor and author of ATCsim project: Francisco Martin
 *
 *  ATCSim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 */

#include "ros/ros.h"

#include "Airport.cpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atcsim_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  Airport airport;

  ros::ServiceServer ss = n.advertiseService("commander_service", &Airport::command, &airport);
  ros::ServiceServer sss = n.advertiseService("info_service", &Airport::sendInfo, &airport);

  std::cerr<< std::endl<<" ---------- ATCSIM IS ON ----------"<< std::endl<<std::endl;



  while (ros::ok())
  {
    airport.doWork();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
