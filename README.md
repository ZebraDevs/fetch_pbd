# Fetch Programming by Demonstration [![Build Status](https://api.travis-ci.org/fetchrobotics/fetch_pbd.png)](https://travis-ci.org/fetchrobotics/fetch_pbd)

TODO: Travis build status and coveralls testing?

This repository is based on PR2 Programming by Demonstration. This version is for the Fetch Robot.

The original [PR2 Programming by Demonstration](https://github.com/PR2/pr2_pbd) was done by [Maya Cakmak](http://www.mayacakmak.com/) and the [Human-Centered Robotics Lab](https://hcrlab.cs.washington.edu/) at the University of Washington.

## System Requirements
This PbD is designed for Ubuntu 14.04 and ROS Indigo.

## Installing from Source
Clone this repository and build on the robot:
```bash
cd ~/catkin_ws/src
git clone https://github.com/fetchrobotics/fetch_pbd.git
cd ~/catkin_ws
catkin_make
```
To make sure all dependencies are installed:
```bash
cd ~/catkin_ws
source ~/devel/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=indigo -y
```
## Making Changes to the Web Interface
To make changes to the web interface, you will want to install all of the web dependencies:
```bash
roscd fetch_pbd_interaction
cd web_interface/fetch-pbd-gui
./install_web_dependencies.sh
```
Note that the above adds the following paths to the end of your `~/.bashrc`:
```bash
export NVM_DIR="/home/USERNAME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"  # This loads nvm
```
You will want to source your `~/.bashrc` file again before running the software.
Now all of the dependencies are installed. After you make changes to the web interface, to build you can run:
```bash 
roscd fetch_pbd_interaction
cd web_interface/fetch-pbd-gui
./build_frontend.sh
```
Now when you roslaunch this software it will run using your newly built changes!

## Running
### Commands on Fetch Robot
#### Terminal #1
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fetch_pbd_interaction pbd.launch
```
You can run the backend without the "social gaze" head movements or without the sounds by passing arguments to the launch file:
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fetch_pbd_interaction pbd.launch social_gaze:=false play_sound:=false
```

You can also pass arguments to the launch file to save your actions to a json file or load them from a json file.
This behaviour is a bit complicated. It is recommended that you specify the full path to files or else it will look in your .ros folder.
If you specify a from_file then actions will be loaded from that file. They will replace the ones in your session database.
Whatever was in your session database will get stored in a timestamped file in your .ros folder (not overwritten).
If you specify a to_file then the session you are starting will be saved to that file.
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fetch_pbd_interaction pbd.launch from_file:=/full/path/from.json to_file:=/full/path/to.json
```

### Using the GUI
In your browser go to ROBOT_HOSTNAME:8080 in your browser to use the GUI. There is a "mobile" interface, which is the same but without the visualization. This can be useful for just saving/deleting primitives. 

The main page lists all the available actions.
![Main page](https://cloud.githubusercontent.com/assets/1470402/17989388/c71a3da2-6ae1-11e6-9d2f-894a67e508ca.png)
You can directly run/copy/delete actions from the main page. Or hit the "Edit" button to see more information on that action.

On the "Current Action" screen, most of the buttons are pretty self-explanatory. You can execute the entire action using the "Run" button at the bottom of the screen. This will execute all of the primitives in the order they appear in the Primitive List. You can click on a specific primitive (either the marker or the list item), to highlight the primitive.
![](https://cloud.githubusercontent.com/assets/1470402/17989398/d0602b2e-6ae1-11e6-8add-edcedf6285b6.png)

You can show/hide the markers for each primitive by clicking the marker icon for the primitive in the Primitive List.
![](https://cloud.githubusercontent.com/assets/1470402/17989394/d05bf02c-6ae1-11e6-9446-9847bbd419ea.png)

You can change the order of the primitives by dragging them to a new position in the list.
![](https://cloud.githubusercontent.com/assets/1470402/17989397/d0608290-6ae1-11e6-98a4-bbb1049e1185.png)

You can edit the position and orientation of certain primitives by clicking the edit icon or by moving the interactive marker.
![](https://cloud.githubusercontent.com/assets/1470402/17989393/d05b87ea-6ae1-11e6-85d7-922c6dc4844a.png)

You can change the frame that certain primitives are relative to by right-clicking the marker.
![](https://cloud.githubusercontent.com/assets/1470402/17989395/d05d657e-6ae1-11e6-8236-459118a70b8a.png)

You can also change the name of the action.
![](https://cloud.githubusercontent.com/assets/1470402/17989396/d05f44d4-6ae1-11e6-9363-f242c5ea15b6.png)

You can record objects in the environment using the "Record Objects" button. The objects will appear in the viewer. Poses can be relative to these objects in the environment.
![](https://user-images.githubusercontent.com/1470402/28444453-2f3502bc-6d72-11e7-9c64-0fc8e03ed9b5.png)
![](https://user-images.githubusercontent.com/1470402/28444447-2c0d509e-6d72-11e7-899c-46247b95446a.png)

### Grasping in Fetch PbD
You can now (optionally) run Fetch PbD with a grasp suggestion service. This is a service that takes in a point cloud for an object and returns a PoseArray of possible grasps for that object. Fetch Pbd provides feedback by publishing which grasp was chosen by the user. Any service that adheres to the .srv interface defined by [SuggestGrasps.srv](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/develop/srv/SuggestGrasps.srv) and optionally the feedback .msg interface, [GraspFeedback.msg](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/develop/msg/GraspFeedback.msg), can be used. If you want to start Fetch PbD with a grasp suggestion service:
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fetch_pbd_interaction pbd.launch grasp_suggestion_service:=grasp_service_name grasp_feedback_topic:=grasp_feedback
```
Then in the interface you can right-click on objects in the scene and add a grasp for that object. Initially a grasp primitive is generated that is just a placeholder and does not specify any poses. To use the grasp suggestion service to generate grasps, you then right click the blue placeholder box and select "Generate grasps". The service will return a list of grasp poses. Fetch PbD sets a pre-grasp that is 15 cm away from the grasp. The first grasp generated is shown and you can switch to other grasp options by right-clicking the grasp marker and selecting one from the list. 

![](https://user-images.githubusercontent.com/1470402/28444632-6e349ec2-6d73-11e7-9b50-1ed4a26292d3.png)
![](https://user-images.githubusercontent.com/1470402/28444631-6e34a084-6d73-11e7-83ca-cbeb900877ab.png)
![](https://user-images.githubusercontent.com/1470402/28444630-6e2cf802-6d73-11e7-9edd-dfa3caf08674.png)

Grasps are executed like any other primitve and can be added to actions in combination with other primitives.

### Code Interface
You can also access the actions you've programmed through code. You still need to run pbd_backend.launch. 

#### Commands on Fetch
```bash
source ~/catkin_ws/devel/setup.bash
rosrun fetch_pbd_interaction demo.py
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

