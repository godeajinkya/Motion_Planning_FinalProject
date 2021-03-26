##Solving Motion Planning Queries for TurtleBot3 using PRM Technique##

## Instructions to run project files ##

# Pre-requisites #

Install the following software on Ubuntu 16.04:

Install Catkin, Python 2.7(code based on python 2.7 due to ROS Kinetic constraints) and Python 3 (only for verification purposes if required).
Install numpy in python environment.

Create Catkin Workspace:
Create catkin workspace named cpsc8810_project in your home directory.

ROS-Kinetic:
$ cd
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh

$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers

TurtleBot3 libraries:

$ cd ~/cpsc8810_project/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

Adding a parameter modified G-mapping code:
Go to directory turtlebot3 --> turtlebot3_slam --> launch
Copy the turtlebot3_gmapping_new.launch file from our provided directory and paste in your directory.

Add pathfinder package to your library:
Copy the pathfinder package folder into your workspace.
$ cd ~/cpsc8810_project/
$ catkin_make

# Main Code #

To launch SLAM and PRM, use the slamprm.launch file:
$ roslaunch pathfinder slamprm.launch
	wait for approximately 60-70 seconds for SLAM to recognize the environment and for prmplanner to generate roadmap and path.
close process using ctrl+C.

To launch the simulation for the path in Gazebo, use navigation.launch file:
$ roslaunch pathfinder navigation.launch
	The simulation starts immediately. To check the co-ordinates of the robot, you can use the following command in anothet terminal window:
$ rostopic echo /odom/pose/pose

The simulation's completion can be checked when the robot stops at the goal point that can be seen using the above command.
Close the process using ctrl+C.

All scripts are placed under pathfinder/src/scripts.
The py_2 suffixed scripts are used in the launch file as ROS uses python 2.
To visually see the roadmap, you can run prmplanner.py in a python 3 environment with numpy installed.

Scripts description (In launch order):
map_generator.py: Generates mapdata.csv from rostopic occupancy grid to generate the obstacle map.
obs_import_py2.py: Imports the obstacle data using the above generate map data and creates prm_obstacle.csv for use in prm.
prmplanner_py2.py: Uses the obstacle data and creates a roadmap and also searches for the best path using A* algorithm. Returns path.csv for further use.
navigation.py: Provides the velocity to the robot in Gazebo to traverse the nodes obtained from path.csv.
rest of the files: Support files with functions and classes.
prmplanner.py: Can be used to generate visible roadmap as this uses python 3 that can run Tkinter.
navigation.py: Takes path
graph.py: support file for PRM.

The path.csv file in this compressed folder is the path file for the submitted video presentation points.