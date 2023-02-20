# Search & Rescue Mission
This is the final project of ENPM809Y. The serach and rescue mission consists of two robots, the explorer and the follower. The explorer, which work on the "searh" part, first navigates to specified locations. At each location, it decodes and locate aruco markers, and then sent marker data to the follower. The follower, working on the "rescue" part of the mission, then navigates to marker locations in an order specified by the marker.
## Installation
1. Clone Search_Rescue_Mission repository into your ROS workspace
```
git clone https://github.com/varithpu/Search_Rescue_Mission.git
```
2. Build package with catkin_make
```
catkin_make
```
## Running the Simulation
1. Launch mission environment, including Gazebo and RViz
```
roslaunch final_project multiple_robots.launch 
```
2. Start the mission
```
rosrun final_project final_project_node 
```
## Screenshot

<img src="https://github.com/varithpu/Search_Rescue_Mission/blob/main/pics/pic1.png" width=100% height=100%>

## Video
https://www.youtube.com/watch?v=EAKBaGLi33k
