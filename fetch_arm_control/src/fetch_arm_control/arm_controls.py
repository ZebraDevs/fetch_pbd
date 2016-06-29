#!/usr/bin/env python

'''This runs the PbD system (i.e. the backend).'''

# Core ROS imports come first.
import rospy
import signal
from fetch_arm_control.arm import Arm
from fetch_arm_control.arm_control_marker import ArmControlMarker
from tf import TransformListener
from std_msgs.msg import String

class ArmControls:
    '''Marker for visualizing the steps of an action.'''

    def __init__(self, realtime=False):
        self.realtime = realtime
    	tf_listener = TransformListener()
        arm = Arm(tf_listener)
    	self.marker = ArmControlMarker(arm)
        rospy.Subscriber('arm_control_reset', String, self.reset_arm_controls)
    	rospy.loginfo('Arm controls initialized.')

    def reset_arm_controls(self, dummy):
    	self.marker.reset()

    def update(self):
        self.marker.update()

        if self.realtime:
                    self.marker.move_to_cb(None)