#!/usr/bin/env python
'''Node to control robot's arms using interactive markers'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy

# ROS builtins
from tf import TransformListener

# Local
from fetch_arm_control.arm import Arm
from fetch_arm_control.arm_control_marker import ArmControlMarker

if __name__ == '__main__':

    # Register as a ROS node.
    rospy.init_node('fetch_arm_interaction', anonymous=True)

    # Run the system
    tf_listener = TransformListener()
    arm = Arm(tf_listener)
    marker = ArmControlMarker(arm)
    rospy.spin()
    # while(not rospy.is_shutdown()):
    #     marker.update()
    #     rospy.sleep(0.05)