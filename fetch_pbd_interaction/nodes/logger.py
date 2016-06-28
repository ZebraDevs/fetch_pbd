#!/usr/bin/env python

# ######################################################################
# Imports
# ######################################################################


# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# System builtins
import os
import sys
import time

# ROS builtins
import rosbag
from std_msgs.msg import String

# ROS 3rd party
from sensor_msgs.msg import JointState
from speakeasy.msg import SpeakEasyTextToSpeech

# Local
from constants import *


# ######################################################################
# Classes
# ######################################################################

class Logger:
    '''Logs information (useful for user studies).

    The following information is tracked:
     - human speech
     - gaze commands
     - gaze targets
     - joint states
    '''

    def __init__(self):
        self.lastJointSaveTime = time.time()
        self.lastGazeSaveTime = time.time()
        self.streamSavingPeriod = 0.5

        self.allJoints = joints[0] + joints[1]

        rospy.loginfo('Logger: Waiting for the logging directory to be set.')

        # Wait for the directory to be specified
        while not rospy.has_param('data_directory'):
            time.sleep(0.01)
        self.logDirectory = rospy.get_param('data_directory')

        rospy.loginfo('Logger: got logging directory: ' + self.logDirectory)
        self.initializeBags()

        # ROBOT JOINTS
        rospy.Subscriber('joint_states', JointState, self.receiveJointStates)

        # SPEECH REC
        rospy.Subscriber('recognized_speech', String, self.receiveSpeechRec)

        # TTS
        rospy.Subscriber(
            "speakeasy_text_to_speech_req",
            SpeakEasyTextToSpeech,
            self.receiveTTS
        )

        # GAZE
        rospy.Subscriber('gaze_command', String, self.receiveGazeCommand)
        rospy.Subscriber('gaze_target', String, self.receiveGazeTarget)

    def initializeBags(self):
        self.speechRecBag = rosbag.Bag(
            self.logDirectory + 'humanSpeech.bag', 'w')
        self.ttsBag = rosbag.Bag(self.logDirectory + 'robotSpeech.bag', 'w')
        self.gazeCommandBag = rosbag.Bag(
            self.logDirectory + 'gazeCommand.bag', 'w')
        self.gazeTargetBag = rosbag.Bag(
            self.logDirectory + 'gazeTarget.bag', 'w')
        self.jointStateBag = rosbag.Bag(
            self.logDirectory + 'jointState.bag', 'w')

    def closeAllBags(self):
        self.speechRecBag.close()
        self.ttsBag.close()
        self.gazeCommandBag.close()
        self.gazeTargetBag.close()
        self.jointStateBag.close()

    def update(self):
        time.sleep(0.02)

    # ##################################################################
    # Callback functions
    # ##################################################################

    def receiveJointStates(self, msg):
        if time.time() - self.lastJointSaveTime > self.streamSavingPeriod:
            armState = JointState()
            armState.header = msg.header
            for i in range(len(msg.name)):
                if (msg.name[i] in self.allJoints):
                    armState.name.append(msg.name[i])
                    armState.position.append(msg.position[i])
            self.jointStateBag.write('arm_states', armState)
            self.lastJointSaveTime = time.time()

    def receiveSpeechRec(self, data):
        self.speechRecBag.write('recognized_speech', data)

    def receiveGazeCommand(self, data):
        self.gazeCommandBag.write('gaze_command', data)

    def receiveGazeTarget(self, data):
        if time.time() - self.lastJointSaveTime > self.streamSavingPeriod:
            self.gazeTargetBag.write('gaze_target', data)
            self.lastJointSaveTime = time.time()

    def receiveTTS(self, req):
        ttsCmd = req.command
        if ttsCmd == 0:
            text = String()
            text.data = req.text
            self.ttsBag.write('robot_speech', text)
        else:
            rospy.logerr(
                "Incorrect command type in received TextToSpeech.msg " +
                "message: " + str(ttsCmd))

if __name__ == '__main__':
    rospy.init_node('pr2_pbd_logger')
    logger = Logger()
    rospy.on_shutdown(logger.closeAllBags)
    rospy.spin()
