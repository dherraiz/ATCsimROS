/*
 * Flight.cpp
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

#include "std_msgs/String.h"

#include "Flight.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#include <GL/gl.h>
#endif

#include "Common.h"

#include <iostream>
#include <string>
#include <math.h>





Flight::~Flight() {
	// TODO Auto-generated destructor stub
}

Flight::Flight(std::string _id,std::string _type, Position _pos, float _bearing, float _inclination, float _speed)
{
	id = _id;
	type = _type;
	pos = _pos;
	bearing_ = _bearing;
	inclination_ = _inclination;

	setSpeed(_speed);	// Through set in order to limit speeds

	route.clear();

	if (type == "A320"){
		points_ = INIT_FLIGHT_POINTS;
	}else if(type == "A380"){
		points_ = INIT_FLIGHT_POINTS*2;
	}
	w_speed = 0.0f;

}

void
Flight::update(float delta_t)
{
	float trans;
	Position CPpos;
	float distXY_CP = 10.0f;

	if(routed())
	{
		float goal_bearing, diff_bearing, new_w, diff_inclination, new_in,goal_inc;

		CPpos = route.front().pos;

		if(CPpos.get_z() <= MAINTAIN_ALT){ // Maintain altitude
			float current_alt = (this->getPosition()).get_z();
			CPpos.set_z(current_alt);
			route.front().pos.set_z(current_alt);
		}

		pos.angles(CPpos, goal_bearing, inclination_);

		goal_bearing = normalizePi(goal_bearing + M_PI);
		diff_bearing = normalizePi(goal_bearing - bearing_);
		new_w = diff_bearing;

		if(fabs(new_w)>MAX_FLIFGT_W) new_w = (fabs(new_w)/new_w) * MAX_FLIFGT_W;

		bearing_ = bearing_ + new_w*delta_t*50;


		diff_inclination = normalizePi(goal_inc -inclination_);
		new_in = diff_inclination;

		if(fabs(new_in)>MAX_FLIFGT_W) new_in = (fabs(new_in)/new_in) * MAX_FLIFGT_W;

		inclination_ = inclination_ + new_in*delta_t*50;

		float goal_speed, diff_speed, acc;

		//Maintain speed while turning
		if(fabs(new_w) < MAX_FLIFGT_W*0.9){
			goal_speed = checkSpeedLimits(route.front().speed);
			if(route.front().speed<0){
			 		goal_speed = speed;
			}
		}else{
			goal_speed = speed;
		}
		acc = (goal_speed - speed);

		if(fabs(acc)>MAX_ACELERATION*DIMENSION_FACTOR*ACC_FACTOR) acc = (acc/fabs(acc))*MAX_ACELERATION*DIMENSION_FACTOR*ACC_FACTOR;

		speed = speed + acc*delta_t*50;


		if(pos.distance(CPpos)<DIST_POINT){
			route.pop_front();
		}


	}else{
		inclination_ = 0.0;
	}

	last_pos = pos;

	trans = speed*0.1 * delta_t;


	pos.set_x(pos.get_x() + trans * cos(bearing_) * cos(inclination_));
	pos.set_y(pos.get_y() + trans * sin(bearing_) * cos(inclination_));
	pos.set_z(pos.get_z() + ( trans * sin(inclination_)));

	points_= points_ - delta_t*10;


}


float Flight::checkSpeedLimits(float tgt_speed){
	return (tgt_speed > CRASH_SPEED_MAX*DIMENSION_FACTOR*ACC_FACTOR ? CRASH_SPEED_MAX*DIMENSION_FACTOR*ACC_FACTOR : tgt_speed);
}
