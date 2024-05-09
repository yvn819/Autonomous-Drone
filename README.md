# Sub-Terrain Challange 

Welcome to the group project for the Autonomous Systems course! In this project, we have developed an autonomous drone system, including environment perception, object identification, obstacle avoidance, and path planning. Through several Robot Operating System (ROS) packages, we enable a self-driving UAV to navigate to a cave, autonomously explore the cave, and eventually identify all target objects.


## Installation

### Prerequisites

The code in this repository was developed and tested on Ubuntu 20.04. It probably will not work on other operating systems or versions.

Before getting started, you may need to install some ROS packages. In a terminal, run:  
```
sudo apt-get install ros-noetic-octomap ros-noetic-octomap-server ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins ros-noetic-map-server
```


### How to use

1. Clone package

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/zang09/AStar-ROS.git
```

2. Build package
```
$ cd ..
$ catkin_make
```

3. Launch package

In the first terminal:
```
$ source devel/setup.bash
$ roslaunch simulation simulation.launch
```

In the second terminal:
```
$ source devel/setup.bash
$ roslaunch filter filter.launch
```

In the third terminal:
```
$ source devel/setup.bash
$ roslaunch controller_pkg explore.launch
```

### Results
![image](https://github.com/yvn819/Autonomous-Drone/blob/main/document/OccupancyGrid.png)

<!-- ## Contributor

- Zihao Wang
    [@wzh_miasanmia]

- Yutong Xin 
    [@00000000014ABFEA] -->





