# Fetch Programming by Demonstration

TODO: Travis build status and coveralls testing?

This repository is a port of PR2 Programming by Demonstration to the Fetch robot.  

The original [PR2 Programming by Demonstration](https://github.com/PR2/pr2_pbd) was done by [Maya Cakmak](http://www.mayacakmak.com/) and the [Human-Centered Robotics Lab](https://hcrlab.cs.washington.edu/) at the University of Washington.

## System Requirements
This PbD is designed for Ubuntu 14.04 and ROS Indigo. You also need [mongo_msg_db](https://github.com/jstnhuang/mongo_msg_db) and [mongo_msg_db_msgs](https://github.com/jstnhuang/mongo_msg_db_msgs).

## Installing
Clone this repository and build on both your desktop machine and on the robot:
```bash
cd ~/catkin_ws/src
git clone https://github.com/jstnhuang/mongo_msg_db_msgs.git
git clone https://github.com/jstnhuang/mongo_msg_db.git
git clone https://github.com/fetchrobotics/sandbox.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=indigo -y
catkin_make
```
Note: The above may not work if other things in the sandbox are broken. Just clone the sandbox somewhere else and copy the fetch_pbd repo ONLY into your workspace.

## Running
### Fetch
#### Commands on Fetch
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fetch_pbd_interaction pbd_backend.launch
```

### Desktop
```bash
roslaunch fetch_pbd_interaction pbd_frontend.launch
```

Plug in a microphone to your computer.
Speak into the microphone to issue speech commands to the robot.
The voice commands are not currently documented.
