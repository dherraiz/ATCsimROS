/*
 * Flight.h
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


#ifndef FLIGHT_H_
#define FLIGHT_H_
#include <string>
#include "Position.h"
#include "Position.cpp"
#include "Common.h"
#include <list>

#include <visualization_msgs/Marker.h>

typedef struct {
	Position pos;
	float speed;
} Route;

class Flight {
public:
	Flight(std::string _id, std::string _type, Position _pos, float _bearing, float _inclination, float _speed);
	virtual ~Flight();

	void update(float delta_t);


	std::list<Route> *getRoute() { return &route;};
	bool routed() { return !route.empty();};
	Position getPosition() { return pos;};
	float getInclination() { return inclination_;};
	float getBearing() { return bearing_;};
	float getSpeed() { return speed;};
	void setSpeed(float tgt_speed) {speed = checkSpeedLimits(tgt_speed);}
	float getPoints() {return points_;};
	std::string getId(){return id;};
	std::string getType(){return type;};



private:
	std::string id;
	std::string type;
	Position pos, last_pos;
	float bearing_, inclination_;
	float speed, w_speed;
	std::list<Route> route;
	float points_;

	float checkSpeedLimits(float tgt_speed);
};


#endif /* FLIGHT_H_ */
