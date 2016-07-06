'''Robot responses in the dialog.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
PKG = 'fetch_pbd_interaction'
import roslib
roslib.load_manifest(PKG)
import rospy

# System builtins
import os

# ROS builtins
from actionlib import SimpleActionClient

# ROS 3rd party
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# Local
from robot_speech import RobotSpeech
from fetch_social_gaze.msg import GazeGoal, GazeAction
from fetch_pbd_interaction.msg import RobotSound

# ######################################################################
# Module level constants
# ######################################################################

# Note the constant PKG above should be set.

# Directory sounds are in, within the package.
SOUNDS_DIR = os.path.join(roslib.packages.get_pkg_dir(PKG), 'sounds', '')

# Common postfix for sound files.
SOUND_FILEFORMAT = '.wav'

# Name of sound file for unrecognized sounds.
SOUND_UNKNOWN = 'OTHER'

# The ROS action for head gazes.
ACTION_GAZE = 'gaze_action'

# ######################################################################
# Classes
# ######################################################################


class Response:
    '''Unit of interaction, explains how to respond to a speech
    command. A new instance is made for each type of response.
    '''

    # Variables that will be set later.
    gaze_client = None
    _sound_client = None
    _robot_speech = None

    # Gaze goals.
    _enable_social_gaze = None
    glance_action = GazeGoal.GLANCE_EE
    follow_action = GazeGoal.FOLLOW_EE

    # Responses (robot speech).
    # Open
    open_response = RobotSpeech.HAND_OPENING
    already_open_response = RobotSpeech.HAND_ALREADY_OPEN

    # Close
    close_response = RobotSpeech.HAND_CLOSING
    already_closed_response = RobotSpeech.HAND_ALREADY_CLOSED

    # Sounds (robot sound).
    all_sounds =  [RobotSound.ALL_POSES_DELETED, RobotSound.ERROR,
                  RobotSound.MICROPHONE_WORKING, RobotSound.POSE_SAVED,
                  RobotSound.START_TRAJECTORY, RobotSound.CREATED_ACTION,
                  RobotSound.EXECUTION_ENDED, RobotSound.OTHER,
                  RobotSound.STARTING_EXECUTION, RobotSound.SUCCESS]

    def __init__(self, function_to_call, function_param=None):
        '''
        Args:
            function_to_call (function): Upon respond(...), the function
                that should be called.
            function_param (Landmark): Upon respond(...), when calling
                function_to_call, the argument that should be passed in.
                Note that unlike almost every other instance of Landmark
                in the code, this could be any python Landmark, not the
                Landmark.msg Landmark.
        '''
        self.function_to_call = function_to_call
        self.function_param = function_param

        if Response.gaze_client is None:
            Response.gaze_client = SimpleActionClient(ACTION_GAZE, GazeAction)
            Response.gaze_client.wait_for_server(rospy.Duration(10))

        if Response._robot_speech is None:
            Response._robot_speech = RobotSpeech()

        if Response._sound_client is None:
            Response._sound_client = SoundClient()

    # ##################################################################
    # Static methods: Public (API)
    # ##################################################################

    @staticmethod
    def perform_gaze_action(gaze_action):
        '''Triggers a gaze action.

        Args:
            gaze_action (int): One of the constants defined in
                Gaze.action.
        '''
        if Response._enable_social_gaze is None:
            Response._enable_social_gaze = rospy.get_param('/enable_social_gaze', True)
        if Response._enable_social_gaze:
            goal = GazeGoal()
            goal.action = gaze_action
            Response.gaze_client.send_goal(goal)

    @staticmethod
    def force_gaze_action(gaze_action):
        '''Triggers a gaze action, even if enable_social_gaze is false.

        Args:
            gaze_action (int): One of the constants defined in
                Gaze.action.
        '''
        goal = GazeGoal()
        goal.action = gaze_action
        Response.gaze_client.send_goal(goal)

    @staticmethod
    def look_at_point(point):
        '''Looks at a specific point.

        Args:
            point (Point)
        '''
        if Response._enable_social_gaze is None:
            Response._enable_social_gaze = rospy.get_param('/enable_social_gaze', True)
        if Response._enable_social_gaze:
            Response.gaze_client.send_goal(GazeGoal(GazeGoal.LOOK_AT_POINT, point))

    @staticmethod
    def say(speech_resp):
        '''Triggers a speech action.

        Args:
            speech_resp (str): What to say.
        '''
        Response._robot_speech.say(speech_resp)

    @staticmethod
    def respond_with_sound(speech_resp):
        '''Triggers a sound response.

        Args:
            speech_resp (str): The speech (concept) that we want to
                convey. Should be one of RobotSpeech.*
        '''
        if speech_resp == RobotSpeech.STEP_RECORDED:
            Response.play_sound(RobotSound.POSE_SAVED)
        elif speech_resp == RobotSpeech.TEST_RESPONSE:
            Response.play_sound(RobotSound.MICROPHONE_WORKING)
        elif speech_resp == RobotSpeech.SKILL_CLEARED:
            Response.play_sound(RobotSound.ALL_POSES_DELETED)
        elif RobotSpeech.START_EXECUTION in speech_resp:
            Response.play_sound(RobotSound.STARTING_EXECUTION)
        elif speech_resp == RobotSpeech.EXECUTION_ENDED:
            Response.play_sound(RobotSound.EXECUTION_ENDED)
        elif speech_resp == RobotSpeech.STARTED_RECORDING_MOTION:
            Response.play_sound(RobotSound.START_TRAJECTORY)
        elif RobotSpeech.SKILL_CREATED in speech_resp:
            Response.play_sound(RobotSound.CREATED_ACTION)
        elif (speech_resp == RobotSpeech.START_STATE_RECORDED or
              speech_resp == RobotSpeech.STOPPED_RECORDING_MOTION or
              RobotSpeech.SWITCH_SKILL in speech_resp):
            Response.play_sound(RobotSound.SUCCESS)
        elif (speech_resp == RobotSpeech.OBJECT_NOT_DETECTED or
              speech_resp == RobotSpeech.MOTION_NOT_RECORDING or
              speech_resp == RobotSpeech.ERROR_NEXT_SKILL or
              speech_resp == RobotSpeech.ERROR_NO_EXECUTION or
              speech_resp == RobotSpeech.ERROR_NO_SKILLS or
              speech_resp == RobotSpeech.ERROR_PREV_SKILL or
              speech_resp == RobotSpeech.EXECUTION_ERROR_NOIK or
              speech_resp == RobotSpeech.EXECUTION_ERROR_NOPOSES or
              speech_resp == RobotSpeech.EXECUTION_PREEMPTED or
              speech_resp == RobotSpeech.HAND_ALREADY_OPEN or
              speech_resp == RobotSpeech.HAND_ALREADY_CLOSED):
            Response.play_sound(RobotSound.ERROR)
        else:
            Response.play_sound(RobotSound.OTHER)

    @staticmethod
    def play_sound(requested_sound):
        '''Plays the requested sound.

        Args:
            requested_sound (str): One of the constants in RobotSound.*.
        '''
        if requested_sound in Response.all_sounds:
            Response._sound_client.playWave(
                os.path.join(SOUNDS_DIR, requested_sound + SOUND_FILEFORMAT))
        else:
            Response._sound_client.playWave(
                os.path.join(SOUNDS_DIR, SOUND_UNKNOWN + SOUND_FILEFORMAT))

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def respond(self):
        '''Triggers the response (that has already been set).'''
        speech_resp, gaze_resp = self.function_to_call(self.function_param)

        rospy.loginfo("Speech response: {}".format(speech_resp))
        # Speech response
        if speech_resp is not None:
            Response.say(speech_resp)
            Response.respond_with_sound(speech_resp)
        # Gaze response
        if gaze_resp is not None:
            Response.perform_gaze_action(gaze_resp)
