/*
 * Airport.cpp
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
#include "atcsim_msgs/CommanderService.h"
#include "atcsim_msgs/InfoService.h"
#include "atcsim_msgs/ArrayFlight.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sstream>
#include <string>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "Airport.h"

geometry_msgs::TransformStamped
generate_flight_obj(Position pos, float bearing, float inclination, std::string name_flight)
{
  tf2::Stamped<tf2::Transform> object;
  object.frame_id_ = "world";
  object.stamp_ = ros::Time::now();

  object.setOrigin(tf2::Vector3(pos.get_x(), pos.get_y(), pos.get_z()));

  tf2::Quaternion q;
  q.setRPY(0, -inclination, bearing);
  object.setRotation(q);

  geometry_msgs::TransformStamped object_msg = tf2::toMsg(object);
  object_msg.child_frame_id = name_flight;

  return object_msg;
}


void
Airport::drawInfo()
{
  float cos45= 0.7071;
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker marker_text_point;
  marker_text_point.header.frame_id = "world";
  marker_text_point.header.stamp = ros::Time::now();
  marker_text_point.ns = "Points";
  marker_text_point.id = 10;
  marker_text_point.action = visualization_msgs::Marker::ADD;
  marker_text_point.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  std::ostringstream buff1;
  buff1<<int(AirportPoints_);
  marker_text_point.text = buff1.str();
  marker_text_point.pose.position.x = 20000;
  marker_text_point.pose.position.y = 50000;
  marker_text_point.pose.position.z = 10000;
  marker_text_point.pose.orientation.x = 0.0;
  marker_text_point.pose.orientation.y = 0.0;
  marker_text_point.pose.orientation.z = 0.0;
  marker_text_point.pose.orientation.w = 1.0;
  marker_text_point.scale.z = 10000;
  marker_text_point.color.r = 1.0f;
  marker_text_point.color.g = 1.0f;
  marker_text_point.color.b = 1.0f;
  marker_text_point.color.a = 1.0;
  marker_array.markers.push_back(marker_text_point);

  visualization_msgs::Marker marker_text_time;
  marker_text_time.header.frame_id = "world";
  marker_text_time.header.stamp = ros::Time::now();
  marker_text_time.ns = "Time";
  marker_text_time.id = 11;
  marker_text_time.action = visualization_msgs::Marker::ADD;
  marker_text_time.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  std::ostringstream buff2;
  float mins = (local_time_- fmod(local_time_,60))/60;
  float secs = fmod(local_time_,60);
  float decs = fmod(fmod(local_time_,60),1);
  if(mins<10 && secs<10){buff2<<"0"<<mins<<":0"<<(secs-decs)<<" (x"<<mult_speed_<<")";}
  else if(mins<10 && secs>=10){buff2<<"0"<<mins<<":"<<(secs-decs)<<" (x"<<mult_speed_<<")";}
  else if(mins>=10 && secs<10){buff2<<mins<<":0"<<(secs-decs)<<" (x"<<mult_speed_<<")";}
  else{buff2<<mins<<":"<<(secs-decs)<<" (x"<<mult_speed_<<")";}

  marker_text_time.text = buff2.str();
  marker_text_time.pose.position.x = 50000;
  marker_text_time.pose.position.y = -20000;
  marker_text_time.pose.position.z = 10000;
  marker_text_time.pose.orientation.x = 0.0;
  marker_text_time.pose.orientation.y = 0.0;
  marker_text_time.pose.orientation.z = 0.0;
  marker_text_time.pose.orientation.w = 1.0;
  marker_text_time.scale.z = 10000;
  marker_text_time.color.r = 1.0f;
  marker_text_time.color.g = 1.0f;
  marker_text_time.color.b = 1.0f;
  marker_text_time.color.a = 1.0;
  marker_array.markers.push_back(marker_text_time);

  airportMarker_pub_.publish(marker_array);
}


void
Airport::drawHelpers(int num)
{
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker aux_marker;
  aux_marker.header.frame_id = "world";
  aux_marker.header.stamp = ros::Time::now();
  aux_marker.ns = "Visual helper P1";
  aux_marker.id = 1;
  aux_marker.action = visualization_msgs::Marker::ADD;
  aux_marker.type = visualization_msgs::Marker::CYLINDER;
  aux_marker.pose.position.x = 1700 +num*0.15*3937*0.304776;
  aux_marker.pose.position.y = -2650 +num*0.15*3937*0.952424147199;
  aux_marker.pose.position.z = 0;
  aux_marker.pose.orientation.x = 0.0;
  aux_marker.pose.orientation.y = 0.0;
  aux_marker.pose.orientation.z = 0.0;
  aux_marker.pose.orientation.w = 1.0;
  aux_marker.scale.x = 250;
  aux_marker.scale.y = 250;
  aux_marker.scale.z = 10;
  aux_marker.color.r = 1.0f;
  aux_marker.color.g = 1.0f;
  aux_marker.color.b = 0.0f;
  aux_marker.color.a = 1.0;
  marker_array.markers.push_back(aux_marker);

  aux_marker.ns = "Visual helper P2";
  aux_marker.id = 2;
  aux_marker.pose.position.x = 3150 +num*0.15*3937*0.304776;
  aux_marker.pose.position.y = -2500 +num*0.15*3937*0.952424147199;
  marker_array.markers.push_back(aux_marker);

  aux_marker.ns = "Visual helper P3";
  aux_marker.id = 3;
  aux_marker.pose.position.x = 1300 +num*0.15*3937*0.354227;
  aux_marker.pose.position.y = -3800 +num*0.15*3937*-0.935159;
  marker_array.markers.push_back(aux_marker);

  aux_marker.ns = "Visual helper P4";
  aux_marker.id = 4;
  aux_marker.pose.position.x = 3150 +num*0.15*3937*0.354227;
  aux_marker.pose.position.y = -3300 +num*0.15*3937*-0.935159;
  marker_array.markers.push_back(aux_marker);

  airportMarker_pub_.publish(marker_array);
}


visualization_msgs::Marker
Airport::drawWp(boost::shared_ptr<Flight> flight, int count)
{
  std::list<Route>::iterator it_wps;
  geometry_msgs::Point aux_point;
  visualization_msgs::Marker aux_marker;
  aux_marker.header.frame_id =  "world";
	aux_marker.header.stamp = ros::Time();
  std::ostringstream buff2;
  buff2<<(flight)->getId()<<"_wps";
	aux_marker.ns = buff2.str();
	aux_marker.id = count;
	aux_marker.type = visualization_msgs::Marker::LINE_STRIP;
	aux_marker.action = visualization_msgs::Marker::ADD;
	aux_marker.pose.position.x = 0;
	aux_marker.pose.position.y = 0;
	aux_marker.pose.position.z = 0;
	aux_marker.pose.orientation.x = 0.0;
	aux_marker.pose.orientation.y = 0.0;
	aux_marker.pose.orientation.z = 0.0;
	aux_marker.pose.orientation.w = 1.0;
  ros::Duration one_second(1);
  aux_marker.lifetime = one_second;
	aux_marker.scale.x = 20;
	aux_marker.color.a = 1.0;
	aux_marker.color.r = 0.0;
	aux_marker.color.g = 0.0;
	aux_marker.color.b = 1.0;

  aux_point.x = flight->getPosition().get_x();
  aux_point.y = flight->getPosition().get_y();
  aux_point.z = flight->getPosition().get_z();
  aux_marker.points.push_back(aux_point);

  for(it_wps = (((flight)->getRoute())->begin()); it_wps!=((flight)->getRoute())->end(); ++it_wps){
    aux_point.x = it_wps->pos.get_x();
    aux_point.y = it_wps->pos.get_y();
    aux_point.z = it_wps->pos.get_z();
    aux_marker.points.push_back(aux_point);
  }
  return aux_marker;
}


visualization_msgs::Marker
Airport::add_marker(std::string id, std::string type, int count, int mir)
{
  visualization_msgs::Marker aux_marker;
  aux_marker.header.frame_id =  id;
	aux_marker.header.stamp = ros::Time();
	aux_marker.ns = id;
	aux_marker.id = count;
	aux_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	aux_marker.action = visualization_msgs::Marker::ADD;
	aux_marker.pose.position.x = 0;
	aux_marker.pose.position.y = 0;
	aux_marker.pose.position.z = 0;
	aux_marker.pose.orientation.x = 0.0;
	aux_marker.pose.orientation.y = 0.0;
	aux_marker.pose.orientation.z = 1.0;
	aux_marker.pose.orientation.w = 1.0;
  ros::Duration one_second(1);
  aux_marker.lifetime = one_second;
  if (type == "A320"){
    aux_marker.scale.x = 2*mir;
  	aux_marker.scale.y = 2;
  	aux_marker.scale.z = 2;
  	aux_marker.color.a = 1.0;
  	aux_marker.color.r = 1.0;
  	aux_marker.color.g = 1.0;
  	aux_marker.color.b = 1.0;
  }else if (type == "A380"){
    aux_marker.scale.x = 5*mir;
  	aux_marker.scale.y = 5;
  	aux_marker.scale.z = 5;
  	aux_marker.color.a = 1.0;
  	aux_marker.color.r = 0.831;
  	aux_marker.color.g = 0.686;
  	aux_marker.color.b = 0.216;
  }
	aux_marker.mesh_resource = "package://using_markers/models/SC_Private_001.dae";

  return aux_marker;
}


visualization_msgs::Marker
Airport::add_marker_sphere_flight(std::string id,std::string type, int count)
{
  visualization_msgs::Marker aux_marker;
  std::ostringstream buff2;
  buff2<<id<<"_sphere";
  aux_marker.header.frame_id =  id;
	aux_marker.header.stamp = ros::Time();

	aux_marker.ns = buff2.str();
	aux_marker.id = count;
	aux_marker.type = visualization_msgs::Marker::SPHERE;
	aux_marker.action = visualization_msgs::Marker::ADD;
	aux_marker.pose.position.x = 0;
	aux_marker.pose.position.y = 0;
	aux_marker.pose.position.z = 0;
	aux_marker.pose.orientation.x = 0.0;
	aux_marker.pose.orientation.y = 0.0;
	aux_marker.pose.orientation.z = 1.0;
	aux_marker.pose.orientation.w = 1.0;
  ros::Duration one_seconds(1);
  aux_marker.lifetime = one_seconds;
  if (type == "A320"){
    aux_marker.scale.x = 400;
  	aux_marker.scale.y = 400;
  	aux_marker.scale.z = 400;
    aux_marker.color.a = 0.5;
  	aux_marker.color.r = 1;
  	aux_marker.color.g = 0.0;
  	aux_marker.color.b = 0.0;
  }else if (type == "A380"){
    aux_marker.scale.x = 600;
  	aux_marker.scale.y = 600;
  	aux_marker.scale.z = 600;
    aux_marker.color.a = 0.5;
    aux_marker.color.r = 0.831;
  	aux_marker.color.g = 0.686;
  	aux_marker.color.b = 0.216;
  }

  return aux_marker;
}


void
Airport::checkCollisions()
{
	if(flights_.empty()) return;

  std::list<boost::shared_ptr<Flight> >::iterator i,j;
	bool removed = false;

	i =  flights_.begin();
	while(i != flights_.end())
	{
		j = i;
		if(j!=flights_.end()) j++;

		while(j != flights_.end())
		{
			if( (*i)->getPosition().distance((*j)->getPosition()) < COLLISION_DISTANCE)
			{
				std::cerr<<"Collision between "<<(*i)->getId()<<" and "<<(*j)->getId()<<std::endl;
				flights_.erase(i);
				flights_.erase(j);
        AirportPoints_ += COLLISION_POINTS;
				return; //Avoid not valid iterator. Only one collision per cycle
			}
			j++;
		}
		i++;
	}
}


void
Airport::checkCrashes()
{
	if(flights_.empty()) return;

  std::list<boost::shared_ptr<Flight> >::iterator it;

	it = flights_.begin();
	while(it != flights_.end())
	{
		if((*it)->getPosition().get_z()<0)
		{
			std::cerr<<"Crash of "<<(*it)->getId()<<std::endl;
			flights_.erase(it);
      AirportPoints_ += CRASH_HEIGHT_POINTS;
		}else if(toDegrees(fabs((*it)->getInclination())) > CRASH_INC)
		{

			std::cerr<<"Crash of "<<(*it)->getId()<<toDegrees(fabs((*it)->getInclination()))<<std::endl;
			flights_.erase(it);
      AirportPoints_ += CRASH_INC_POINTS;
		}else if( (*it)->getSpeed()<CRASH_SPEED*ACC_FACTOR)
		{
			std::cerr<<"Crash of "<<(*it)->getId()<<std::endl;
			flights_.erase(it);
      AirportPoints_ += CRASH_SPEED_POINTS;
		}else
			it++;
	}
}


void
Airport::checkLandings()
{
	if(flights_.empty()) return;

  std::list<boost::shared_ptr<Flight> >::iterator it;

  float cos45= 0.7071;

  Position posR1(1700,-2650,0);
  Position posR2(3150,-2500,0);
  Position posR3(1300,-3800,0);
  Position posR4(3150,-3300,0);

  float landing_bear_R1R2 = -1.88;
  float landing_bear_R3R4 = 1.93;

	it = flights_.begin();

	while(it != flights_.end())
	{

		if((posR1.distance((*it)->getPosition()) < LANDING_DIST) &&
				(toDegrees(normalizePi(fabs((*it)->getBearing() - landing_bear_R1R2)))<LANDING_BEAR_MAX_ERROR) &&
				((*it)->getSpeed()<LANDING_SPEED*DIMENSION_FACTOR*ACC_FACTOR))
		{
			std::cerr<<"Flight "<<(*it)->getId()<<" landed in P1"<<std::endl;
		  flights_.erase(it);
      AirportPoints_ += (*it)->getPoints();
			return;
		}else if((posR2.distance((*it)->getPosition()) < LANDING_DIST) &&
				(toDegrees(normalizePi(fabs((*it)->getBearing() - landing_bear_R1R2)))<LANDING_BEAR_MAX_ERROR) &&
				((*it)->getSpeed()<LANDING_SPEED*DIMENSION_FACTOR*ACC_FACTOR))
		{
			std::cerr<<"Flight "<<(*it)->getId()<<" landed in P2"<<std::endl;
		  flights_.erase(it);
      AirportPoints_ += (*it)->getPoints();
			return;
		}if((posR3.distance((*it)->getPosition()) < LANDING_DIST) &&
				(toDegrees(normalizePi(fabs((*it)->getBearing() - landing_bear_R1R2)))<LANDING_BEAR_MAX_ERROR) &&
				((*it)->getSpeed()<LANDING_SPEED*DIMENSION_FACTOR*ACC_FACTOR))
		{
			std::cerr<<"Flight "<<(*it)->getId()<<" landed in P3"<<std::endl;
		  flights_.erase(it);
      AirportPoints_ += (*it)->getPoints();
			return;
		}else if((posR4.distance((*it)->getPosition()) < LANDING_DIST) &&
				(toDegrees(normalizePi(fabs((*it)->getBearing() - landing_bear_R1R2)))<LANDING_BEAR_MAX_ERROR) &&
				((*it)->getSpeed()<LANDING_SPEED*DIMENSION_FACTOR*ACC_FACTOR))
		{
			std::cerr<<"Flight "<<(*it)->getId()<<" landed in P4"<<std::endl;
		  flights_.erase(it);
      AirportPoints_ += (*it)->getPoints();
			return;
		}else
			it++;
	}
}


void
Airport::doWork()
{

  std::list<boost::shared_ptr<Flight> >::iterator it;
  std::list<boost::shared_ptr<visualization_msgs::Marker> >::iterator it_marker;
  std::list<Route>::iterator it_wps;

  atcsim_msgs::ArrayFlight arrayflight_pub;
  atcsim_msgs::Flight flight_pub;
  atcsim_msgs::Waypoint wp_pub;

  ros::Publisher arrayflight_info_pub_;
  arrayflight_info_pub_ = n_.advertise<atcsim_msgs::ArrayFlight>("arrayflight_info", 1000);

  ros::Rate loop_rate(10);

  drawInfo();
  checkCollisions();
  checkCrashes();
  checkLandings();


  if ((ros::Time::now() - obj_ts_2).toSec() > INC_SIMTIME) /// RUNAWAYS HELPER DRAWING
  {
    obj_ts_2 = ros::Time::now();
    drawHelpers(num_mark_);
    if(num_mark_>0)
    {
      num_mark_--;
    }else{
      num_mark_=10;
    }
  }



  if ((ros::Time::now() - obj_ts_).toSec() > INC_SIMTIME*mult_speed_/50) ///FLIGHT POSITIONS UPDATING AND INFO PUBLISHING
  {
    local_time_ += INC_SIMTIME*mult_speed_;
    obj_ts_ = ros::Time::now();
    arrayflight_pub.array_flight.clear();

    for(it = flights_.begin(); it!=flights_.end(); ++it) // Info publishing
    {
      (*it)->update(INC_SIMTIME*mult_speed_/50);

      flight_pub.id = (*it)->getId();
      flight_pub.type = (*it)->getType();
      flight_pub.posx = (*it)->getPosition().get_x();
      flight_pub.posy = (*it)->getPosition().get_y();
      flight_pub.posz = (*it)->getPosition().get_z();
      flight_pub.speed = (*it)->getSpeed();
      flight_pub.bearing = (*it)->getBearing();
      flight_pub.inclination = (*it)->getInclination();
      flight_pub.points = (*it)->getPoints();

      flight_pub.wps.clear();
      for(it_wps = (((*it)->getRoute())->begin()); it_wps!=((*it)->getRoute())->end(); ++it_wps){

        wp_pub.x = it_wps->pos.get_x();
        wp_pub.y = it_wps->pos.get_y();
        wp_pub.z = it_wps->pos.get_z();
        wp_pub.speed = it_wps->speed;

        flight_pub.wps.push_back(wp_pub);
      }

      arrayflight_pub.array_flight.push_back(flight_pub);
    }

    arrayflight_info_pub_.publish(arrayflight_pub);
  }///--------------------------------------------------------------------------


  for(it = flights_.begin(); it!=flights_.end(); ++it)///-- TRANSFORM PUBLISHING
  {

    flight_obj_msg = generate_flight_obj((*it)->getPosition(), (*it)->getBearing(), (*it)->getInclination(), (*it)->getId());

    try
    {
      flight_obj_msg.header.stamp = ros::Time::now();
      br.sendTransform(flight_obj_msg);

    }
    catch(tf2::TransformException &exception)
    {
      ROS_ERROR("04");
      ROS_ERROR("%s", exception.what());
    }

  }///--------------------------------------------------------------------------


  ///------------------------------------------------- FLIGHT MARKERS PUBLISHING
  visualization_msgs::MarkerArray marker_array;

  int count = 0;
  for(it = flights_.begin(); it!=flights_.end(); ++it)
  {
    marker_array.markers.push_back(add_marker((*it)->getId(),(*it)->getType(),count++,1));
    marker_array.markers.push_back(add_marker((*it)->getId(),(*it)->getType(),count++,-1));
    marker_array.markers.push_back(add_marker_sphere_flight((*it)->getId(),(*it)->getType(),count++));
    marker_array.markers.push_back(drawWp(*it,count++));

  }

  flightMarkers_pub_.publish(marker_array);
  ///---------------------------------------------------------------------------

}



bool
Airport::command(atcsim_msgs::CommanderService::Request& req,
         atcsim_msgs::CommanderService::Response& res)
{
  bool ach = false;
  std::list<boost::shared_ptr<Flight> >::iterator it;
  std::list<Route>::iterator it_wps;

  if (req.code == 1) ///----------------------------------------------ADD FLIGHT
  {
    ach = true;
    for(it = flights_.begin(); it!=flights_.end(); ++it) //IDs conflicts checking
    {
      if (((*it)->getId()) == req.id){
        ach = false;
      }
    }

    if(ach)
    {
      Position ipos(req.posx, req.posy, req.posz);
      boost::shared_ptr<Flight> aux(new Flight(req.id, req.type, ipos, req.bearing, req.inclination, req.speed));

      for(int i=0; i< req.wps.size(); ++i)
      {
        Position pos0(req.wps[i].x, req.wps[i].y, req.wps[i].z);
        Route r0;

        r0.pos = pos0;
        r0.speed = req.wps[i].speed;
        aux->getRoute()->push_back(r0);

      }
      flights_.push_back(aux);


    }

    res.achieved = ach;
     if(ach){
       ROS_INFO_STREAM(req.id <<" added to the list");
     }else{
       res.expl = "Flight " + req.id + " already exists";
       return 1;
     }


    return true;
  }else if(req.code == 2) ///-------------------------------------------- ADD WP
  {
      ach = false;
      for(it = flights_.begin(); it!=flights_.end(); ++it)
      {
        if (((*it)->getId()) == req.id){
          //ROS_INFO("%f,%f,%f,%f",req.wps[1].x,req.wps[1].y,req.wps[1].z)
          Position pos0(req.wps[0].x, req.wps[0].y, req.wps[0].z);
          Route r0;
          r0.pos = pos0;
          r0.speed = req.wps[0].speed;
          (*it)->getRoute()->push_back(r0);
          ach = true;
        }
      }
      res.achieved = ach;
       if(ach){
         ROS_INFO_STREAM("Waypoint: ["<< req.wps[1].x<<", "<<
                         req.wps[1].y<< " , "<<req.wps[1].z<<
                         "] speed: "<<req.wps[1].speed <<" added to the list of "<<req.id);
       }else{
         res.expl = "Flight " + req.id + " not found";
         return 1;
       }

      return true;
  }else if(req.code == 3) ///------------------------------------------CLEAR WPS
  {
    for(it = flights_.begin(); it!=flights_.end(); ++it)
    {
      if (((*it)->getId()) == req.id){
        (*it)->getRoute()->clear();
        ach = true;
      }
    }
    res.achieved = ach;
    if(ach){
      ROS_INFO_STREAM("Erased all the wps of "<<req.id);
    }else{
       res.expl = "Flight " + req.id + " not found";
       return 1;
    }
    return true;
  }else if(req.code == 4) ///--------------------------------------REMOVE FLIGHT
  {
    ach = false;
    std::list<boost::shared_ptr<Flight> >::iterator it;

  	it = flights_.begin();
  	while(it != flights_.end())
  	{

      if (((*it)->getId()) == req.id){
        flights_.erase(it);
        ach = true;
        it = flights_.end();
      }else{
          it++;
      }
    }
    if(ach){
      ROS_INFO_STREAM("Erased flight "<<req.id);
      res.achieved = ach;
      return 1;
    }else{
       res.expl = "Flight " + req.id + " not found";
       return 1;
    }
    return true;

  }else if(req.code == 5) ///---------------------------REPLACE SPEED MULTIPLIER
  {
    set_mult_speed(req.speed_req);
    res.achieved = true;
    return 1;
  }else{
    return 0;
  } ///-------------------------------------------------------------------------



}


bool
Airport::sendInfo(atcsim_msgs::InfoService::Request& req,
         atcsim_msgs::InfoService::Response& res)
{
  bool ach = false;
  std::list<boost::shared_ptr<Flight> >::iterator it;
  std::list<Route>::iterator it_wps;

  atcsim_msgs::Flight flight_pub;
  atcsim_msgs::Waypoint wp_pub;

  if (req.code == 1)
  {
    for(it = flights_.begin(); it!=flights_.end(); ++it)
    {

      flight_pub.id = (*it)->getId();
      flight_pub.type = (*it)->getType();
      flight_pub.posx = (*it)->getPosition().get_x();
      flight_pub.posy = (*it)->getPosition().get_y();
      flight_pub.posz = (*it)->getPosition().get_z();
      flight_pub.speed = (*it)->getSpeed();
      flight_pub.bearing = (*it)->getBearing();
      flight_pub.inclination = (*it)->getInclination();
      flight_pub.points = (*it)->getPoints();

      flight_pub.wps.clear();
      for(it_wps = (((*it)->getRoute())->begin()); it_wps!=((*it)->getRoute())->end(); ++it_wps){

        wp_pub.x = it_wps->pos.get_x();
        wp_pub.y = it_wps->pos.get_y();
        wp_pub.z = it_wps->pos.get_z();
        wp_pub.speed = it_wps->speed;

        flight_pub.wps.push_back(wp_pub);
      }

      res.flights.push_back(flight_pub);
      ach = true;
    }

    res.achieved = ach;
    return 1;

  }else if(req.code == 2){
    res.points = AirportPoints_;
    res.achieved = true;
    return 1;
  }else if(req.code == 3){
    res.time = local_time_;
    res.achieved = true;
    return 1;
  }else if(req.code == 4){
    res.speed = mult_speed_;
    res.achieved = true;
    return 1;
  }else{
    return 0;
  } ///-------------------------------------------------------------------------

}
