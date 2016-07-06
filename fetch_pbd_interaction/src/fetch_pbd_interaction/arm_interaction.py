#!/usr/bin/env python

# Core ROS imports come first.
import rospy
import signal
from fetch_arm_interaction.arm import Arm
from fetch_pbd_interaction.arm_control_marker import ArmControlMarker
from tf import TransformListener
from std_msgs.msg import String

class ArmInteraction:

    def __init__(self, realtime=False):
        self.realtime = realtime
    	tf_listener = TransformListener()
        arm = Arm(tf_listener)
    	self.marker = ArmControlMarker(arm)
        rospy.Subscriber('arm_interaction_reset', String, self.reset_arm_marker)
    	rospy.loginfo('Arm controls initialized.')

    def reset_arm_marker(self, dummy):
    	self.marker.reset()

    def update(self):
        self.marker.update()

        if self.realtime:
                self.marker.move_to_cb(None)