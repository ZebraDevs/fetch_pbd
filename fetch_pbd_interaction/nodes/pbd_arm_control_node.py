#!/usr/bin/env python
'''Provides a service interface to control the robot's arm 
in the context of PbD.
'''
# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# Local
from fetch_pbd_interaction import ArmControl

# ######################################################################
# Main Function
# ######################################################################

if __name__ == '__main__':

    # Register as a ROS node.
    rospy.init_node('fetch_arm_control', anonymous=True)

    arm_control = ArmControl()

    while(not rospy.is_shutdown()):
        arm_control.update()
        # This is the pause between update runs. Note that this doesn't
        # guarantee an update rate, only that there is this amount of
        # pause between udpates.
        rospy.sleep(0.1)
