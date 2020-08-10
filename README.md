## Brief introduction(temporary version)
-----
### Prerequisite
* ROS Melodic
    For installation please refer to [ros oﬃcial website](http://wiki.ros.org/melodic/Installation/Ubuntu)
### Set up
   1. If you installed catkin via apt-get for ROS Melodic, you need to source its setup script ﬁrst  
   ```
    $ source /opt/ros/melodic/setup.bash
   ```
   1. Create and build a catkin workspace
   ```
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/
   ```
   1. clone this repo to `~/catkin_ws/src` and checkout branch ***ros-arena***
   2. build the project by using `catkin_make` and make sure you are in the root directory of your catkin workspace
### How to use
1. use `ros_launch arena2d arena` to run the arena2d simulator.
2. `cd build && make run_tests` to run unittests.