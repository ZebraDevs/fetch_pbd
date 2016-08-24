# Fetch Programming by Demonstration

TODO: Travis build status and coveralls testing?

This repository is based on PR2 Programming by Demonstration. This version is for the Fetch Robot.

The original [PR2 Programming by Demonstration](https://github.com/PR2/pr2_pbd) was done by [Maya Cakmak](http://www.mayacakmak.com/) and the [Human-Centered Robotics Lab](https://hcrlab.cs.washington.edu/) at the University of Washington.

## System Requirements
This PbD is designed for Ubuntu 14.04 and ROS Indigo.

## Installing
Clone this repository and build on both your desktop machine and on the robot:
```bash
cd ~/catkin_ws/src
git clone https://github.com/fetchrobotics/sandbox.git
cd ~/catkin_ws
catkin_make
```
Note: The above may not work if other things in the sandbox are broken. Just clone the sandbox somewhere else and copy the fetch_pbd repo ONLY into your workspace.

You also need NodeJs, npm, bower and Polymer installed. To install Nodejs and npm, go [here](https://nodejs.org/en/). npm is installed with Nodejs. Then:
```bash
npm install -g bower
npm install -g polymer-cli
cd /path_to_fetch_pbd/fetch_pbd_interaction/web_interface/fetch-pbd-gui
bower install
```

## Running
### Commands on Fetch Robot
#### Terminal #1
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fetch_pbd_interaction pbd_backend.launch
```
You can run the backend without the "social gaze" head movements or without the sounds by passing arguments to the launch file:
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fetch_pbd_interaction pbd_backend.launch social_gaze:=false play_sound:=false
```

#### Terminal #2
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fetch_pbd_interaction pbd_web_frontend.launch
```
#### Terminal #3
This will open the web interface.
```bash
cd /path_to_fetch_pbd/fetch_pbd_interaction/web_interface/fetch-pbd-gui
polymer serve --hostname 0.0.0.0
```

### Code Interface
You can also access the actions you've programmed through code. You still need to run pbd_backend.launch (and you can run the frontend stuff too if you want; it won't hurt). Check out the examples folder.
#### Commands on Fetch
```bash
source ~/catkin_ws/devel/setup.bash
rosrun fetch_pbd_interaction test_session_interface.py
```

## System Overview
### Interaction Node
The pbd_interaction_node.py handles the interaction between speech/GUI and the rest of the system. Changes happen through the update loop in interaction.py and also through the callbacks from speech/GUI commands. interaction.py also subscribes to updates from the pbd_world_node.py, which notifies it of changes in objects in the world. Through callbacks and the update loop, interaction.py hooks in to session.py. session.py handles creating actions and primitives and saving them to the database.

### Arm Control Node
The pbd_arm_control_node.py is how the robot's arm is controlled to execute actions/primitives. It provides a lower level service interface to move the arm. The interaction node interacts with this through the interface in robot.py.

### World Node
The pbd_world_node.py handles the robot's perception of the world. Other nodes ask the world node about the state of the world and can both send and subscribe to updates to the world. Its main function is to provide a list of objects currently in the scene.

### Social Gaze Node
The social_gaze_server.py handles the movements of the robot's head. This is also controlled through the robot.py interface. The sounds are also provided through this interface.

