#!/usr/bin/env python
'''This runs the "core" of the PbD backend.
It handles the interactions between the GUI and the state of the world
and recorded actions.
It coordinates responses from the head and arms of the robot.
'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# ROS builtins
from visualization_msgs.msg import MarkerArray
from interactive_markers.interactive_marker_server import \
     InteractiveMarkerServer
from tf import TransformListener
import rospkg

# Local
from fetch_arm_control.msg import GripperState
from fetch_pbd_interaction.session import Session
from fetch_pbd_interaction.msg import ExecutionStatus
from fetch_pbd_interaction.srv import Ping, PingResponse, GetObjectList
from fetch_pbd_interaction.msg import RobotSound, WorldState
from fetch_pbd_interaction.robot import Robot
from std_msgs.msg import String
from std_srvs.srv import Empty

# ######################################################################
# Module level constants
# ######################################################################

EXECUTION_Z_OFFSET = -0.00
BASE_LINK = 'base_link'
TOPIC_IM_SERVER = 'programmed_actions'


# ######################################################################
# Main Function
# ######################################################################

if __name__ == '__main__':

    # Register as a ROS node.
    rospy.init_node('fetch_session_interface_demo', anonymous=True)

    # Run the system
    tf_listener = TransformListener()
    robot = Robot(tf_listener)
    im_server = InteractiveMarkerServer(TOPIC_IM_SERVER)
    # Get path to example json file
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('fetch_pbd_interaction') + \
                "/examples/example_actions.json"
    session = Session(robot, tf_listener,
                      im_server, from_file=file_path)

    if session.n_actions() > 0:
        rospy.loginfo("There are {} actions.".format(session.n_actions()))

        # Select first action
        session.switch_to_action_by_index(0)
        # Execute first action
        rospy.loginfo("Executing action 0, " +
                      "with {} primitives.".format(session.n_primitives()))
        session.execute_current_action()

        # You can specify pauses between actions
        rospy.sleep(3)

        # You can also execute individual primitives
        if session.n_primitives() > 0:
            rospy.loginfo("Executing first primitive of first action")
            session.execute_primitive(0)
        else:
            rospy.loginfo("Action 0 has no primitives.")

        action_num = raw_input('Which action should we execute next? Enter 1 or 2:')
        action_num = int(action_num)

        rospy.loginfo("Executing action {}.".format(action_num))
        session.switch_to_action_by_index(action_num)
        session.execute_current_action()

        # You can also summon actions by name
        rospy.loginfo("Executing Wave action")
        session.switch_to_action_by_name("Wave")
        session.execute_current_action()

    else:
        rospy.logwarn("No actions available!")

