# ATCsimROS
TFG Daniel García Herriaz ETSIT, URJC. Madrid 2020

#REQUISITOS: 
C++ compiler, ROS and its 'Kinetic Kame' distribution are necesary.

To create a workspace
  /catkin_ws/src
  /catkin_ws/
  mkdir -p
  cd
  catkin_make
  source /opt/ros/kinetic/setup.bash
  
To clone the repository
  git clone https://github.com/dherraiz/ATCsimROS.git
  
To compile the program 
  cd /catkin_ws/
  catkin_make
  
To run the program
  roslaunch atcsim demo.launch
