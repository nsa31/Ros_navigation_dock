# Project : Egg-hunting-Robot_ROS
The project is done at University of Alberta (UoA) for the course CMPUT 412: Experimental Robotics.

## Overview
The robot builds a map of the environment, moves along the map from a randomly placed location, finds the visual targets placed on the walls, and docks infront of the target while playing a unique sound to the target type.  


## Dependencies
We tested our project on the following environment.
* Ubuntu 14.04
* Python 2.7.6
* ROS Indigo
* Numpy
* OpenCV-Python 2.4.8
 

How to configure Joy:
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

## How to run 
### Collect map 
See previous Project: SLAM-Robot Using ROS-Kobuki Turtlebot for a complete description on how to perform GMapping for map collection.
In addition, the costmap was modified to prevent the robot from moving outside of the map (i.e.: down the hallway or the washroom areas) when localization fails. 
(IMG: MYMAP_CS10B)

### Localization
Unlike the previous project where the robot's initial position could be manually set via the 2D pose estimate in RVIZ, this project requires the robot to search for its initial position. Luckily, the AMCL package includes a global localization rosservice call. This facilitates the dispersion of all particles randomly through the free space in the map, so that AMCL can search for the robot's location on the entire map. 
Then, two localization methods were tested. The first method involves the robot performing a gradual outward spiral motion and random walk to make particles converge. The second method involves the robot performing stationary spinning motion to make particles converge. The exit condition for both methods are based on a fix time constraint (time stamp). After several tests, it was found that the second method outperforms the first in terms of its accuracy and rate of success. Therefore, the second method is implemented. After the robot is localized, navigation starts. 

### Navigation
The same actionlib as the previous project is used. However, instead of 4 waypoints on the map, 17 waypoints are set and sent to the navigation stack. The reason for an increase in the number of waypoints is to cover the 5 walls in a symmetric manner using 1 RGB camera for vision. The symmetric manner means that if the robot localizes in the wrong half region of the map, it can still cover all walls correctly without missing any possible target locations (note: walls on opposite ends are non-symmetric, i.e.: one side has door the other doesnt). In addition, if 2 cameras are used instead of 1, where the other camera faces the side wall, then 6 waypoints should be sufficient. However, because of computer compatability issue, the laptop was unable to support a second camera feature. When using Kernel 3.13, a second camera can be supported with the laptop, but sound and internet was lost. 
Then, after the robot performs initial localization, it travels to the nearest waypoint and starts to loop through the remaining 16 waypoints, while searching for targets on the wall.  
(IMG: MAP WITH 17 WAYPOINTS)

### Target Detection
#### UA logo: Template Matching
#### AR logo: AR_track_alvar
#### Face-wall at 90 degrees

### Docking

### Improvement

### Trial after the competition

### Performance Video

### Discussion

### Future Work

## Authors

* [Nazmus Sakib](https://github.com/nsa31)
* **Vivian Ting**
## Acknowledgement 

* [Programming Robots with ROS](https://github.com/osrf/rosbook/blob/master)

