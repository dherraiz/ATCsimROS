cmake_minimum_required(VERSION 3.0.2)
project(atcsim)


find_package(catkin REQUIRED COMPONENTS
geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  roslint
  roscpp
  rospy
  std_msgs
  atcsim_msgs
  message_generation
  atcsim_msgs

)



## Generate messages in the 'msg' folder
 add_message_files(
   FILES
 )




## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   atcsim_msgs	
 )




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES atcsim
#  CATKIN_DEPENDS roscpp rospy std_msgs
   CATKIN_DEPENDS message_runtime
   CATKIN_DEPENDS atcsim_msgs
#  DEPENDS system_lib
)


## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(commander src/commander.cpp)
target_link_libraries(commander ${catkin_LIBRARIES} readline )

add_executable(generator src/generator.cpp)
target_link_libraries(generator ${catkin_LIBRARIES} )


add_executable(atcsim_node src/atcsim.cpp)
target_link_libraries(atcsim_node ${catkin_LIBRARIES} )
