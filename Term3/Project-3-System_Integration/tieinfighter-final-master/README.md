# Team TieInFighter

## Team Members and Focus
* Amol Sahasrabudhe (amol.sahasrabudhe@gmail.com) - TL classifier
* Siraj Haque (siraj_79@hotmail.com) - TL classifier
* Jason Kang (jacquestkirk@gmail.com) - Waypoint Updater
* Shobhit K (shobhitkukreti@yahoo.com) - Control by DBW
* Daniel Tang (daniel.z.tang@gmail.com) - System Integration

## Introduction

For this project, our team designed a fully autonomous vehicle system, initially to be tested out on a simulator, and then on Udacity’s real self-driving car. The project can be broken up into 4 parts: (1) traffic light detection, (2) control by drive-by-wire (DBW), (3) Waypoint updater/follower, and (4) system integration.

## Traffic Light Detection

The traffic light detector node publishes at 10Hz, and it contains 3 main tasks:

* Find the closest traffic light to the car's position
* Find the state of the traffic light
* If the closest light is red, publish its waypoint to the /traffic_waypoint topic

The detailed implementation of the traffic classifier can be found in team member's report.

## Control by DBW
Drive By Wire technology elimiantes the use of hydraulic systems  and brings in the Electronic Control System for control. The DBW Node subscribes to the twist_cmd topic and uses a combination of Yaw Controller, PID Controller and a low pass filter to generate Throttle, Brake and Steering Commands. These commands are then published on the topics 
/vehicle/throttle_cmd, /vhecile/brake_cmd and /vehicle/steering_cmd. 

The DBW Node has a fail-safe check called "dbw_enabled" which is published on /vehicle/dbw_enabled topic. This boolean value decides if the DBW Node should be activated or not. It is necessary to differentiate between autonomous mode and the manual mode when a safety driver may take control of the car. 

## Waypoint Updater/Follower

The waypoint updater monitors pose and the set of base waypoints. From this information, it determines the 100 waypoints in front of the vehicle. Scipy's KDTree library is used to search the full set of waypoints efficiently. These 100 waypoints will eventually be pushed to the /final_waypoints topic, where it can be used to display in the simulator. 

The final waypoints also contain information on what speed the vehicle should travel at. To do this, the code monitors /traffic_waypoints in order to determine at what waypoint the vehicle needs to stop. If a stop is imminent, the waypoint updater, decelerates the car by reducing the target speed for subsequent waypoints. 

## System Integration

Finally, all pieces from team members (including traffic light detection, control by DBW, and waypoint updater) are integrated and tested in the simulator on the follow HW/SW configurations:

* HW
    * Intel Core i7-7700HW CPU @ 2.8GHz x 8
    * Quadro M1200
    * 7.5GiB memory
* SW
    * ubuntu 14.04 LTS
    * ROS indigo

### Development Environment Setup

To avoid any risk of incompatiblity of final integration and real world testing, our team decide to use Ubuntu 16.04 Xenial Xerus with [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) for our dev environment. DBW is also downloaded from [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros). Finally the simulator is downloaded at [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
