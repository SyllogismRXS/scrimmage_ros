# SCRIMMAGE ROS

This repository provides an example for using the
[SCRIMMAGE simulator](https://github.com/gtri/scrimmage) with the
[ROS 2D Navigation Stack](http://wiki.ros.org/navigation). The interface
between SCRIMMAGE and ROS is almost identical to the interface between
[stage\_ros](http://wiki.ros.org/stage_ros) and ROS.

## Prerequisites

You will need to build the SCRIMMAGE simulator

## Build

1. Setup a ROS catkin workspace, if you don't have one already:

        $ mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/
        $ catkin_make

2. Clone this repo into into ~/catkin_ws/src:

        $ cd ~/catkin_ws/src
        $ git clone https://github.com/SyllogismRXS/scrimmage_ros.git
        
3. Build it:

        $ cd ~/catkin_ws
        $ catkin_make
        
## Run Example

The example SCRIMMAGE mission / roslaunch simulates two ground robots running
the ROS 2D Navigation stack in a simple maze world. It's probably a good idea
to open a new terminal at this point, so that your terminal has access to any
new SCRIMMAGE environment variables.

    $ source ~/catkin_ws/devel/setup.bash               # source the catkin_ws
    $ roslaunch scrimmage_ros multiple-robots.launch    # launch scrimmage/ros
    
The SCRIMMAGE simulator controls the time, generates sensor data, and
integrates the motion models. To kill the simulation, type "CTRL + c" in the
window that you ran the roslaunch command from.

## Notes

### Robot Initial Positions

The robots will be initialized using the positions in the scrimmage mission
file (e.g., ros-ex1.xml), but you can specify an initial pose estimate for each
robot in the multiple-robots.launch file. You will want to modify the initial
pose estimates to match the actual poses in the scrimmage mission file. This
may be automated in the future, but it's manual configuration for now.

### Map File

You need to specify the same map file in both multiple-robots.launch
(name="map\_name" default="simple-maze-1") and the ros-ex1.xml scrimmage
mission file (e.g., entity_interaction order="0" map="simple-maze-1").

### Robot Transforms

You don't need to use an URDF file for a simple ground robot with a few
sensors. Specify the sensor transform from the robot's center using the xyz and
rpy XML attribute tags of a sensor tag.
