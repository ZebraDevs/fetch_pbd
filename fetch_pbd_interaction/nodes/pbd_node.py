#!/usr/bin/env python
'''This runs the PbD system (i.e. the backend).'''

# Core ROS imports come first.
import rospy
import signal
import fetch_pbd_interaction
from fetch_pbd_interaction import ActionDatabase
from fetch_pbd_interaction import ArmControl
from fetch_pbd_interaction import ExecuteActionServer
from fetch_pbd_interaction import Interaction
from fetch_pbd_interaction import Session
from fetch_pbd_interaction import World
from fetch_pbd_interaction.srv import ExecuteActionById


def signal_handler(signal, frame):
    # The following makes sure the state of a user study is saved, so that it can be recovered
    global interaction
    interaction.session.save_current_action()
    rospy.loginfo("Saved experiment state. Terminating.")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGQUIT, signal_handler)

UPDATE_WAIT_SECONDS = 0.1

if __name__ == '__main__':
    global interaction
    # Register as a ROS node.
    rospy.init_node('fetch_pbd_interaction', anonymous=True)

    # Run the system
    db = ActionDatabase.build_real()
    world = World()
    session = Session(world.get_frame_list(), db)
    arm_control = ArmControl(World.tf_listener)
    interaction = Interaction(arm_control, session, world)
    execute_server = ExecuteActionServer(interaction)
    # This service does not get called in normal interaction
    rospy.Service('execute_action', ExecuteActionById, execute_server.serve)

    while (not rospy.is_shutdown()):
        interaction.update()
        # This is the pause between update runs. Note that this doesn't
        # guarantee an update rate, only that there is this amount of
        # pause between udpates.
        rospy.sleep(UPDATE_WAIT_SECONDS)
