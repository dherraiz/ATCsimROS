/*
 * generator.cpp
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
#include "atcsim_msgs/CommanderService.h"

#include <sys/time.h>
#include <math.h>
#include "Common.h"
#include "Position.cpp"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "generator");

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<atcsim_msgs::CommanderService>("commander_service");

  atcsim_msgs::CommanderService srv;


  ros::Time obj_ts = ros::Time::now();

  float time_generation = 10;
  int num = 0;
  bool primero =true;
  int type_count =0;
  std::string type;

  std::cerr<< std::endl<<" ------- FLIGHT GENERATION IS ON -------"<< std::endl<<std::endl;



  while (ros::ok())
  {



    if ((ros::Time::now() - obj_ts).toSec() > time_generation)
    {
      type_count++;
      if ((type_count%3)==0){
        type = "A380";
      }else{
        type = "A320";
      }

      obj_ts = ros::Time::now();


      float angle, x, y, z;
    	float bear, inc;
    	char id[6];

    	angle = toRadians((float)(rand() % 360-180));

    	x = (AIRPORT_DISTANCE_MAX * cos(angle) ) ;
    	y = AIRPORT_DISTANCE_MAX * sin(angle) ;
    	z = (FLIGHT_HEIGHT + (float)(rand() % 2000 ));

    	Position ipos(x, y, z);
    	Position pos0(2500.0, -3000.0, 0.0);

    	pos0.angles(ipos, bear, inc);

    	sprintf(id, "IB%4.4d", num++);

      srv.request.code = 1;
      srv.request.id = id;
      srv.request.type = type;
      srv.request.posx = x;
      srv.request.posy =  y;
      srv.request.posz =  z;
      srv.request.speed =  250*DIMENSION_FACTOR*ACC_FACTOR;
      srv.request.bearing =  bear;
      srv.request.inclination =  0;

      int runaway;

      if (bear<=0 && bear >= -1.88)
      {
        runaway =1;
      }else if(bear<=-1.88 && bear >= -3.14159)
      {
        runaway =2;
      }else if(bear>=0 && bear <= 1.93)
      {
        runaway =3;
      }else
      {
        runaway =4;
      }

      if (primero){
        primero = false;
        runaway = 3;
      }

      //------------------------------------------------------------LANDING WPS:
      if(runaway == 1){
        atcsim_msgs::Waypoint wp_req;
        srv.request.wps.clear();
        wp_req.x = 1700 +4*3937*0.304776;
        wp_req.y = -2650 +4*3937*0.952424147199;
        wp_req.z = 3000;
        wp_req.speed = 250*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1700 +2*3937*0.304776;
        wp_req.y = -2650 +2*3937*0.952424147199;
        wp_req.z = 1200;
        wp_req.speed = 250*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1700 +1*3937*0.304776;
        wp_req.y = -2650 +1*3937*0.952424147199;
        wp_req.z = 500;
        wp_req.speed = 200*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1700 +0.5*3937*0.304776;
        wp_req.y = -2650 +0.5*3937*0.952424147199;
        wp_req.z = 0;
        wp_req.speed = 200*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1700+0.2*3937*0.304776;
        wp_req.y = -2650+0.2*3937*0.952424147199;
        wp_req.z = 0;
        wp_req.speed = 150*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1700;
        wp_req.y = -2650;
        wp_req.z = 0;
        wp_req.speed = 90*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
      }else if(runaway ==2){
        atcsim_msgs::Waypoint wp_req;
        srv.request.wps.clear();
        wp_req.x = 3150 +4*3937*0.304776;
        wp_req.y = -2500 +4*3937*0.952424147199;
        wp_req.z = 3000;
        wp_req.speed = 250*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3150 +2*3937*0.304776;
        wp_req.y = -2500 +2*3937*0.952424147199;
        wp_req.z = 1200;
        wp_req.speed = 250*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3150 +1*3937*0.304776;
        wp_req.y = -2500 +1*3937*0.952424147199;
        wp_req.z = 500;
        wp_req.speed = 200*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3150 +0.5*3937*0.304776;
        wp_req.y = -2500 +0.5*3937*0.952424147199;
        wp_req.z = 0;
        wp_req.speed = 200*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3150+0.2*3937*0.304776;
        wp_req.y = -2500+0.2*3937*0.952424147199;
        wp_req.z = 0;
        wp_req.speed = 150*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3150;
        wp_req.y = -2500;
        wp_req.z = 0;
        wp_req.speed = 90*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
      }else if(runaway ==3){
        atcsim_msgs::Waypoint wp_req;
        srv.request.wps.clear();
        wp_req.x = 1300 +4*3937*0.354227;
        wp_req.y = -3800 +4*3937*-0.935159;
        wp_req.z = 3000;
        wp_req.speed = 250*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1300 +2*3937*0.354227;
        wp_req.y = -3800 +2*3937*-0.935159;
        wp_req.z = 1200;
        wp_req.speed = 250*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1300 +1*3937*0.354227;
        wp_req.y = -3800 +1*3937*-0.935159;
        wp_req.z = 500;
        wp_req.speed = 200*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1300 +0.5*3937*0.354227;
        wp_req.y = -3800 +0.5*3937*-0.935159;
        wp_req.z = 0;
        wp_req.speed = 200*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1300 +0.2*3937*0.354227;
        wp_req.y = -3800 +0.2*3937*-0.935159;
        wp_req.z = 0;
        wp_req.speed = 150*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1300 ;
        wp_req.y = -3800;
        wp_req.z = 0;
        wp_req.speed = 90*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
      }else if(runaway ==4){
        atcsim_msgs::Waypoint wp_req;
        srv.request.wps.clear();
        wp_req.x = 3150 +4*3937*0.354227;
        wp_req.y = -3300 +4*3937*-0.935159;
        wp_req.z = 3000;
        wp_req.speed = 250*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3150 +2*3937*0.354227;
        wp_req.y = -3300 +2*3937*-0.935159;
        wp_req.z = 1200;
        wp_req.speed = 250*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3150 +1*3937*0.354227;
        wp_req.y = -3300 +1*3937*-0.935159;
        wp_req.z = 500;
        wp_req.speed = 200*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3150 +0.5*3937*0.354227;
        wp_req.y = -3300 +0.5*3937*-0.935159;
        wp_req.z = 0;
        wp_req.speed = 200*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3150 +0.2*3937*0.354227;
        wp_req.y = -3300 +0.2*3937*-0.935159;
        wp_req.z = 0;
        wp_req.speed = 150*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3150 ;
        wp_req.y = -3300;
        wp_req.z = 0;
        wp_req.speed = 90*DIMENSION_FACTOR*ACC_FACTOR;
        srv.request.wps.push_back(wp_req);
      }
      ROS_INFO_STREAM("NEW FLIGHT:  "<< id);

      if (client.call(srv))
        {

          if(!(srv.response.achieved))
            ROS_INFO_STREAM("FAIL: "<<srv.response.expl);

        }
      else
        {
          ROS_ERROR("Failed to call service");
          return 1;
        }

    }
  }
return 0;
}
