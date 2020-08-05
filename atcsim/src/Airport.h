/*
 * Airport.h
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
#include <iostream>
#include <list>
#include <memory>
#include <vector>
#include <string>

#include "Flight.cpp"

class Airport {
public:
	Airport(): n_() {

    obj_ts_ = ros::Time::now();
		obj_ts_2 = ros::Time::now();
		arrayflight_info_pub_ = n_.advertise<atcsim_msgs::ArrayFlight>("arrayflight_info", 1000);
		flightMarkers_pub_ = n_.advertise<visualization_msgs::MarkerArray>( "visualization_flight_marker", 0 );
		airportMarker_pub_ = n_.advertise<visualization_msgs::MarkerArray>( "visualization_airport_marker", 0 );

		AirportPoints_ = 0;
		mult_speed_ =1;
		local_time_=0;
		num_mark_ =0;

  };
  void doWork();
	void checkCollisions();
	void checkCrashes();
	void checkLandings();
	void set_mult_speed(float mult){
		if (mult < 0.5){ mult = 0.5;}
		if (mult > 5)  { mult = 5;}
		mult_speed_ = mult;
	}

	bool command(atcsim_msgs::CommanderService::Request& req,
					atcsim_msgs::CommanderService::Response& res);

	bool sendInfo(atcsim_msgs::InfoService::Request& req,
	       	atcsim_msgs::InfoService::Response& res);

	visualization_msgs::Marker add_marker(std::string id, std::string type, int count, int mir);
	visualization_msgs::Marker add_marker_sphere_flight(std::string id, std::string type, int count);
	visualization_msgs::Marker drawWp(boost::shared_ptr<Flight> flight, int count);


	void drawInfo();
	void drawHelpers(int num);

private:

	std::list<boost::shared_ptr<Flight> > flights_;

	ros::Time obj_ts_;
	ros::Time obj_ts_2;
	int num_mark_;

  ros::NodeHandle n_;
	ros::Publisher arrayflight_info_pub_;
	ros::Publisher airportMarker_pub_;
	ros::Publisher flightMarkers_pub_;

	tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped flight_obj_msg;

	float AirportPoints_;
	float mult_speed_;
	float local_time_;





};
