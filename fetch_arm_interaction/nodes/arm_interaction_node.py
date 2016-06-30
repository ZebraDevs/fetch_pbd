#!/usr/bin/env python

'''This runs the PbD system (i.e. the backend).'''

# Core ROS imports come first.
import rospy
import signal
from fetch_arm_interaction.arm_interaction import ArmInteraction


if __name__ == '__main__':

    # Register as a ROS node.
    rospy.init_node('fetch_arm_interaction', anonymous=True)

    # Run the system
    realtime = False
    arm_interaction = ArmInteraction(realtime)
    while(not rospy.is_shutdown()):
        arm_interaction.update()
        rospy.sleep(0.1)
