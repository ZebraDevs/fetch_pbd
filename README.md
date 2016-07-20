# Fetch Programming by Demonstration

TODO: Travis build status and coveralls testing?

This repository is a port of PR2 Programming by Demonstration to the Fetch robot.

The original [PR2 Programming by Demonstration](https://github.com/PR2/pr2_pbd) was done by [Maya Cakmak](http://www.mayacakmak.com/) and the [Human-Centered Robotics Lab](https://hcrlab.cs.washington.edu/) at the University of Washington.

## System Requirements
This PbD is designed for Ubuntu 14.04 and ROS Indigo.

## Installing
Clone this repository and build on both your desktop machine and on the robot:
```bash
cd ~/catkin_ws/src
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

## System Overview
### Interaction Node
The pbd_interaction_node.py handles the interaction between speech/GUI and the rest of the system. Changes happen through the update loop in interaction.py and also through the callbacks from speech/GUI commands. interaction.py also subscribes to updates from the pbd_world_node.py, which notifies it of changes in objects in the world. Through callbacks and the update loop, interaction.py hooks in to session.py. session.py handles creating actions and primitives and saving them to the database.

### Arm Control Node
The pbd_arm_control_node.py is how the robot's arm is controlled to execute actions/primitives. It provides a lower level service interface to move the arm. The interaction node interacts with this through the interface in robot.py.

### World Node
The pbd_world_node.py handles the robot's perception of the world. Other nodes ask the world node about the state of the world and can both send and subscribe to updates to the world. Its main function is to provide a list of objects currently in the scene.

### Social Gaze Node
The social_gaze_server.py handles the movements of the robot's head. This is also controlled through the robot.py interface. The sounds are also provided through this interface.

