/*
 * commander.cpp
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
#include "atcsim_msgs/InfoService.h"


#include <stdio.h>
#include <readline/readline.h>

#include <readline/history.h>

#include "Flight.cpp"
#include <readline/readline.h>
#include <readline/history.h>
#include <stdlib.h>
#include <iostream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "commander");

  ros::NodeHandle n;

  ros::ServiceClient clientCommand = n.serviceClient<atcsim_msgs::CommanderService>("commander_service");
  ros::ServiceClient clientinfo = n.serviceClient<atcsim_msgs::InfoService>("info_service");

  atcsim_msgs::CommanderService srv_command;
  atcsim_msgs::InfoService srv_info;

  atcsim_msgs::Waypoint wp_req;


  std::cout << std::endl<< " _____________________________________________ " << std::endl;
  std::cout << "|                                             |" << std::endl;
  std::cout << "|                A T C  S I M                 | " << std::endl;
  std::cout << "|                                             |" << std::endl;
  std::cout << "|_____________________________________________|" << std::endl;

  std::cout << "    _______         ______         __       __                                " << std::endl;
  std::cout << "   /   __  \\       /  __  \\       |   \\   /   |          " << std::endl;
  std::cout << "  /  /   \\__\\     /  /  \\  \\      |    \\ /    |        " << std::endl;
  std::cout << " |  /            |  /    \\  |     |  |\\   /|  |           " << std::endl;
  std::cout << " | |             | |      | |     |  | \\_/ |  |            " << std::endl;
  std::cout << " |  \\     __     |  \\    /  |     |  |     |  |           " << std::endl;
  std::cout << "  \\  \\___/ /      \\  \\__/  /      |  |     |  |         " << std::endl;
  std::cout << "   \\______/        \\______/       |__|     |__|           " << std::endl;

  std::cout << "   __       __         ___         __       __              " << std::endl;
  std::cout << "  |   \\   /   |       /   \\       |   \\    |  |          " << std::endl;
  std::cout << "  |    \\ /    |      /     \\      |    \\   |  |          " << std::endl;
  std::cout << "  |  |\\   /|  |     /  /_\\  \\     |  |\\ \\  |  |        " << std::endl;
  std::cout << "  |  | \\_/ |  |    /   ___   \\    |  | \\ \\ |  |         " << std::endl;
  std::cout << "  |  |     |  |   /   /   \\   \\   |  |  \\ \\|  |         " << std::endl;
  std::cout << "  |  |     |  |  |   /     \\   |  |  |   \\    |           " << std::endl;
  std::cout << "  |__|     |__|  |__|       |__|  |__|    \\ __|            " << std::endl;

  std::cout << "   _________       __________       ________                " << std::endl;
  std::cout << "  |   ____  \\     |   _______|     |   ___  \\             " << std::endl;
  std::cout << "  |  |    \\  \\    |  |             |  |   \\  \\          " << std::endl;
  std::cout << "  |  |     \\  |   |  |___          |  |___/  /             " << std::endl;
  std::cout << "  |  |      | |   |   ___|         |      __/               " << std::endl;
  std::cout << "  |  |     /  |   |  |             |  |\\  \\               " << std::endl;
  std::cout << "  |  |____/  /    |  |_______      |  | \\  \\              " << std::endl;
  std::cout << "  |_________/     |__________|     |__|  \\__\\             " << std::endl<< std::endl<< std::endl<< std::endl;





  bool finish = false;
  bool ok;
  while (!finish) {

    char * line = readline("--> ");


    char* token = strtok(line, (char*) " ");

    if (strcmp(token, "flight") == 0){////--------------------------------FLIGHT
      token = strtok(NULL,  (char*) " ");
      if (strcmp(token, "list") == 0){////---------------------------flight list
        srv_info.request.code=1;
        if (clientinfo.call(srv_info))
          {
            if(!(srv_info.response.achieved))
              ROS_INFO_STREAM("FAIL: "<<srv_info.response.expl);

            for(int i=0; i< srv_info.response.flights.size(); ++i){
              std::cout << srv_info.response.flights[i].id << " points: "<<srv_info.response.flights[i].points<< '\n';
            }
            std::cout << '\n';
          }
        else
          {
            ROS_ERROR("Failed to call service");
            return 1;
          }
      }else if (strcmp(token, "id") == 0){////-------------------------flight id
        char * id_req = strtok(NULL,  (char*) " ");
        token = strtok(NULL,  (char*) " ");
        if (strcmp(token, "get") == 0){///-------------------------flight id get
          srv_info.request.code=1;
          if (clientinfo.call(srv_info)){
              bool exists = false;
              if(!(srv_info.response.achieved))
                ROS_INFO_STREAM("FAIL: "<<srv_info.response.expl);

              for(int i=0; i< srv_info.response.flights.size(); ++i){
                if (srv_info.response.flights[i].id == id_req){
                  exists = true;
                  std::cout << srv_info.response.flights[i].id <<'\n';
                  std::cout << "Type:"<< srv_info.response.flights[i].type<<'\n';
                  std::cout << "Position:"<<'\n';
                  std::cout <<'\t' << "x: "<<srv_info.response.flights[i].posx <<'\n';
                  std::cout <<'\t' << "y: "<<srv_info.response.flights[i].posy <<'\n';
                  std::cout <<'\t' << "z: "<<srv_info.response.flights[i].posz <<'\n';
                  std::cout << "Speed: "<<srv_info.response.flights[i].speed <<'\n';
                  std::cout << "Bearing: "<<srv_info.response.flights[i].bearing <<'\n';
                  std::cout << "Inclination: "<<srv_info.response.flights[i].inclination <<'\n';
                  std::cout << "Points: "<<srv_info.response.flights[i].points <<'\n';

                }
                if(!exists){std::cout <<"ERROR: Flight "<<id_req<<" not found."<<'\n'<<'\n';}
              }
              std::cout << '\n';
          }else{
            ROS_ERROR("Failed to call service");
            return 1;
          }
        }else if (strcmp(token, "rm") == 0){///---------------------flight id rm
          srv_command.request.code = 4;
          srv_command.request.id = id_req;
          if (clientCommand.call(srv_command)){
              if(!(srv_command.response.achieved))
                ROS_INFO_STREAM("FAIL: "<<srv_command.response.expl);

          }else{
            ROS_ERROR("Failed to call service");
            return 1;
          }
        }else if (strcmp(token, "wp") == 0){///---------------------flight id wp
          token = strtok(NULL,  (char*) " ");
          if (strcmp(token, "get") == 0){//---------------------flight id wp get
            srv_info.request.code=1;
            if (clientinfo.call(srv_info)){
                bool exists = false;
                if(!(srv_info.response.achieved)){
                  ROS_INFO_STREAM("FAIL: "<<srv_info.response.expl);
                  return 1;
                }

                for(int i=0; i< srv_info.response.flights.size(); ++i){
                  if (srv_info.response.flights[i].id == id_req){
                    exists = true;

                    if (srv_info.response.flights[i].wps.size()!= 0){std::cout << "Waypoints: "<<'\n';}
                    for(int j=0; j< srv_info.response.flights[i].wps.size(); ++j)
                    {
                      std::cout <<"[Wp "<<j+1<<"]" <<'\n';
                      std::cout <<'\t' << "x: "<<srv_info.response.flights[i].wps[j].x <<'\n';
                      std::cout <<'\t' << "y: "<<srv_info.response.flights[i].wps[j].y <<'\n';
                      std::cout <<'\t' << "z: "<<srv_info.response.flights[i].wps[j].z <<'\n';
                      std::cout <<'\t' << "Speed: "<<srv_info.response.flights[i].wps[j].speed <<'\n'<<'\n';
                    }
                  }
                }
                if(!exists){std::cout <<"ERROR: Flight "<<id_req<<" not found."<<'\n'<<'\n';}
                std::cout << '\n';
            }else{
              ROS_ERROR("Failed to call service");
              return 1;
            }
          }else if (strcmp(token, "rm") == 0){//-----------------flight id wp rm
            srv_command.request.code = 3;
            srv_command.request.id = id_req;
            if (clientCommand.call(srv_command)){
                if(!(srv_command.response.achieved)){
                  ROS_INFO_STREAM("FAIL: "<<srv_command.response.expl);
                  return 1;
                }else{
                  std::cout << "Waypoints cleared"<< std::endl<< std::endl;
                }

                std::cout << '\n';
            }else{
              ROS_ERROR("Failed to call service");
              return 1;
            }
          }else if (strcmp(token, "set") == 0){//---------------flight id wp set
            char * x = strtok(NULL,  (char*) " ");
            char * y = strtok(NULL,  (char*) " ");
            char * z = strtok(NULL,  (char*) " ");
            token = strtok(NULL,  (char*) " ");
            atcsim_msgs::Waypoint wp_req;
            wp_req.x = atof(x);
            wp_req.y = atof(y);
            wp_req.z = atof(z);
            if(token != NULL){
              wp_req.speed = atof(token);
            }else{
              wp_req.speed = -1;
            }
            srv_command.request.wps.clear();
            srv_command.request.code = 2;
            srv_command.request.id = id_req;
            srv_command.request.wps.push_back(wp_req);
            if (clientCommand.call(srv_command)){
                if(!(srv_command.response.achieved)){
                  ROS_INFO_STREAM("FAIL: "<<srv_command.response.expl);
                  return 1;
                }else{
                  std::cout << "Waypoint added"<< std::endl<< std::endl;
                }

                std::cout << '\n';
            }else{
              ROS_ERROR("Failed to call service");
              return 1;
            }
          }else{
            ROS_INFO_STREAM("FAIL: wrong command: "<< token<<'\n');
          }

        }else{
          ROS_INFO_STREAM("FAIL: wrong command: "<< token<<'\n');
        }
      }else if (strcmp(token, "add") == 0){////-----------------------flight add
        char * id = strtok(NULL,  (char*) " ");
        srv_command.request.code = 1;
        srv_command.request.id = id;
        char * type = strtok(NULL,  (char*) " ");
        if (type != NULL ){
          srv_command.request.type = type;
          char * x = strtok(NULL,  (char*) " ");
          if(x != NULL){
            char * y = strtok(NULL,  (char*) " ");
            char * z = strtok(NULL,  (char*) " ");
            srv_command.request.posx = atof(x);
            srv_command.request.posy = atof(y);
            srv_command.request.posz = atof(z);
            Position ipos(atof(x), atof(y), atof(z));
            Position pos0(0.0, 0.0, 0.0);
            float bear, inc;
            pos0.angles(ipos, bear, inc);
            char * speed = strtok(NULL,  (char*) " ");
            if(speed != NULL){
              srv_command.request.speed = atof(speed);
              char * bearing = strtok(NULL,  (char*) " ");
              if(bearing != NULL){
                srv_command.request.bearing = atof(bearing);
                char * inclination = strtok(NULL,  (char*) " ");
                if(inclination != NULL){
                  srv_command.request.inclination = atof(inclination);
                }else{
                  srv_command.request.inclination = 0;
                }
              }else{
                srv_command.request.bearing = bear;
                srv_command.request.inclination = 0;
              }
            }else{
              srv_command.request.speed = 200;
              srv_command.request.bearing = bear;
              srv_command.request.inclination = 0;
            }
          }else{
            float angle, x, y, z;
            float bear, inc;

            angle = toRadians((float)(rand() % 360 -0));

            x = (AIRPORT_DISTANCE_MAX * cos(angle) ) ;
            y = AIRPORT_DISTANCE_MAX * sin(angle) ;
            z = (FLIGHT_HEIGHT + (float)(rand() % 2000 ));

            Position ipos(x, y, z);
            Position pos0(2500.0, -3000.0, 0.0);

            pos0.angles(ipos, bear, inc);
            srv_command.request.posx = x;
            srv_command.request.posy = y;
            srv_command.request.posz = z;
            srv_command.request.speed =  250*DIMENSION_FACTOR*ACC_FACTOR;
            srv_command.request.bearing =  bear;
            srv_command.request.inclination =  0;
          }
        }else{
          float angle, x, y, z;
          float bear, inc;

          angle = toRadians((float)(rand() % 360 -0));

          x = (AIRPORT_DISTANCE_MAX * cos(angle) ) ;
          y = AIRPORT_DISTANCE_MAX * sin(angle) ;
          z = (FLIGHT_HEIGHT + (float)(rand() % 2000 ));

          Position ipos(x, y, z);
          Position pos0(2500.0, -3000.0, 0.0);

          pos0.angles(ipos, bear, inc);
          srv_command.request.type = "A320";
          srv_command.request.posx = x;
          srv_command.request.posy = y;
          srv_command.request.posz = z;
          srv_command.request.speed =  250*DIMENSION_FACTOR*ACC_FACTOR;
          srv_command.request.bearing =  bear;
          srv_command.request.inclination =  0;
        }
        if (clientCommand.call(srv_command)){
          if(!(srv_command.response.achieved)){
            ROS_INFO_STREAM("FAIL: "<<srv_command.response.expl);
            return 1;
          }else{
            std::cout << "Flight added"<< std::endl<< std::endl;
          }

          std::cout << '\n';
        }else{
          ROS_ERROR("Failed to call service");
          return 1;
        }
      }else{
        ROS_INFO_STREAM("FAIL: wrong command: "<< token<<'\n');
      }


    }else if(strcmp(token, "airport") == 0){////-------------------------AIRPORT
      token = strtok(NULL,  (char*) " ");
      if (strcmp(token, "points") == 0){////----------------------Airport points
        srv_info.request.code=2;
        if (clientinfo.call(srv_info)){
          if(!(srv_info.response.achieved)){
            ROS_INFO_STREAM("FAIL: "<<srv_info.response.expl);
          }else{
            std::cout << " Points:" <<srv_info.response.points<< '\n';
          }

          std::cout << '\n';
        }else{
        ROS_ERROR("Failed to call service");
        return 1;
        }
      }else if (strcmp(token, "time") == 0){////--------------------Airport time
        srv_info.request.code=3;
        if (clientinfo.call(srv_info)){
          if(!(srv_info.response.achieved)){
            ROS_INFO_STREAM("FAIL: "<<srv_info.response.expl);
          }else{
            std::cout << " Time:" <<srv_info.response.time<< '\n';
          }

          std::cout << '\n';
        }else{
          ROS_ERROR("Failed to call service");
          return 1;
        }
      }else if (strcmp(token, "speed") == 0){////------------------Airport speed
        token = strtok(NULL,  (char*) " ");
        if (strcmp(token, "get") == 0){///---------------------Airport speed get
          srv_info.request.code=4;
          if (clientinfo.call(srv_info)){
            if(!(srv_info.response.achieved)){
              ROS_INFO_STREAM("FAIL: "<<srv_info.response.expl);
            }else{
              std::cout << " Speed multiplier: x" <<srv_info.response.speed<< '\n';
            }

            std::cout << '\n';
          }else{
            ROS_ERROR("Failed to call service");
            return 1;
          }
        }else if (strcmp(token, "set") == 0){///---------------Airport speed set
          token = strtok(NULL,  (char*) " ");
          if(token != NULL){
            srv_command.request.code=5;
            srv_command.request.speed_req= atof(token);
            if (clientCommand.call(srv_command)){
                if(!(srv_command.response.achieved))
                  ROS_INFO_STREAM("FAIL: "<<srv_command.response.expl);

            }else{
              ROS_ERROR("Failed to call service");
              return 1;
          }
        }else{
        ROS_INFO_STREAM("FAIL: wrong command: "<< token<<'\n');
        }
      }else{
        ROS_INFO_STREAM("FAIL: wrong command: "<< token<<'\n');
      }
    }else{
      ROS_INFO_STREAM("FAIL: wrong command: "<< token<<'\n');
    }
    }else{
        ROS_INFO_STREAM("FAIL: wrong command: "<< token<<'\n');
    }
  }

  return 0;

}
