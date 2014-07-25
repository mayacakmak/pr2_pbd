''' Robot responses in the dialog '''
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from RobotSpeech import RobotSpeech
from pr2_social_gaze.msg import GazeGoal, GazeAction
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import os, time
from pr2_pbd_interaction.msg import RobotSound
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


class Response:
  "Unit of interaction, explains how to respond to a speech command"
  # Static stuff
  gaze_client = None
  _sound_client = None
  _robot_speech = None
  _marker_publisher = None
  _sounds_dir = roslib.packages.get_pkg_dir('pr2_pbd_interaction')
  _sounds_dir = os.path.join(_sounds_dir, 'sounds', '')
  glance_actions = [GazeGoal.GLANCE_RIGHT_EE, GazeGoal.GLANCE_LEFT_EE]
  follow_actions = [GazeGoal.FOLLOW_RIGHT_EE, GazeGoal.FOLLOW_LEFT_EE]
  open_responses = [RobotSpeech.RIGHT_HAND_OPENING,
                      RobotSpeech.LEFT_HAND_OPENING]
  close_responses = [RobotSpeech.RIGHT_HAND_CLOSING,
                       RobotSpeech.LEFT_HAND_CLOSING]
  release_responses = [RobotSpeech.RIGHT_ARM_RELEASED,
                         RobotSpeech.LEFT_ARM_RELEASED]
  hold_responses = [RobotSpeech.RIGHT_ARM_HOLDING,
                      RobotSpeech.LEFT_ARM_HOLDING]
  already_open_responses = [RobotSpeech.RIGHT_HAND_ALREADY_OPEN,
                            RobotSpeech.LEFT_HAND_ALREADY_OPEN]
  already_closed_responses = [RobotSpeech.RIGHT_HAND_ALREADY_CLOSED,
                              RobotSpeech.LEFT_HAND_ALREADY_CLOSED]
  already_holding_responses = [RobotSpeech.RIGHT_ARM_ALREADY_HOLDING,
                               RobotSpeech.LEFT_ARM_ALREADY_HOLDING]
  already_released_responses = [RobotSpeech.RIGHT_ARM_ALREADY_RELEASED,
                                RobotSpeech.LEFT_ARM_ALREADY_RELEASED]

  all_sounds = [RobotSound.ALL_POSES_DELETED, RobotSound.ERROR,
                 RobotSound.MICROPHONE_WORKING, RobotSound.POSE_SAVED,
                 RobotSound.START_TRAJECTORY, RobotSound.CREATED_ACTION,
                 RobotSound.EXECUTION_ENDED, RobotSound.OTHER,
                 RobotSound.STARTING_EXECUTION, RobotSound.SUCCESS]

  def __init__(self, function_to_call, function_param):
    self.function_to_call = function_to_call
    self.function_param = function_param

    if (Response.gaze_client == None):
      Response.gaze_client = SimpleActionClient('gaze_action',
                                                       GazeAction)
      Response.gaze_client.wait_for_server()

      if (Response._robot_speech == None):
        Response._robot_speech = RobotSpeech()

      if (Response._sound_client == None):
        Response._sound_client = SoundClient()

      if (Response._marker_publisher == None):
        Response._marker_publisher = rospy.Publisher('visualization_marker', Marker)

  def respond(self):
    ''' Triggers the defined response'''
    self.function_to_call(self.function_param)

  @staticmethod
  def perform_gaze_action(gaze_action, wait=False):
    ''' Triggers a gaze action'''
    goal = GazeGoal()
    goal.action = gaze_action
    Response.gaze_client.send_goal(goal)

    if wait:
      time.sleep(0.5)
      rospy.loginfo('Will wait for the gaze action to complete!')
      while (Response.gaze_client.get_state() == GoalStatus.ACTIVE or
             Response.gaze_client.get_state() == GoalStatus.PENDING):
        time.sleep(0.1)


  @staticmethod
  def look_at_point(point):
    ''' Looks at a specific point'''
    Response.gaze_client.send_goal(GazeGoal(GazeGoal.LOOK_AT_POINT, point))

  @staticmethod
  def say(speech_resp):
    ''' Triggers a speech action'''
    Response._sound_client.say(speech_resp)
    Response._say_in_rviz(speech_resp)

  @staticmethod
  def _say_in_rviz(text):
      ''' Visualizes the text that is uttered by the robot in rviz'''
      marker = Marker(type=Marker.TEXT_VIEW_FACING, id=1000,
                 lifetime=rospy.Duration(1.5),
                 pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                 scale=Vector3(0.06, 0.06, 0.06),
                 header=Header(frame_id='base_link'),
                 color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=text)
      Response._marker_publisher.publish(marker)

  @staticmethod
  def respond_with_sound(speech_resp):
    ''' Triggers a sound response'''
    if (speech_resp == RobotSpeech.STEP_RECORDED):
        Response.play_sound(RobotSound.POSE_SAVED)
    elif (speech_resp == RobotSpeech.TEST_RESPONSE):
        Response.play_sound(RobotSound.MICROPHONE_WORKING)
    elif (speech_resp == RobotSpeech.SKILL_CLEARED):
        Response.play_sound(RobotSound.ALL_POSES_DELETED)
    elif (RobotSpeech.START_EXECUTION in speech_resp):
        Response.play_sound(RobotSound.STARTING_EXECUTION)
    elif (speech_resp == RobotSpeech.EXECUTION_ENDED):
        Response.play_sound(RobotSound.EXECUTION_ENDED)
    elif (speech_resp == RobotSpeech.STARTED_RECORDING_MOTION):
        Response.play_sound(RobotSound.START_TRAJECTORY)
    elif (RobotSpeech.SKILL_CREATED in speech_resp):
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
          speech_resp == RobotSpeech.ERROR_NOT_IN_EDIT or
          speech_resp == RobotSpeech.ERROR_PREV_SKILL or
          speech_resp == RobotSpeech.EXECUTION_ERROR_NOIK or
          speech_resp == RobotSpeech.EXECUTION_ERROR_NOPOSES or
          speech_resp == RobotSpeech.EXECUTION_PREEMPTED or
          speech_resp == RobotSpeech.RIGHT_HAND_ALREADY_OPEN or
          speech_resp == RobotSpeech.LEFT_HAND_ALREADY_OPEN or
          speech_resp == RobotSpeech.RIGHT_HAND_ALREADY_CLOSED or
          speech_resp == RobotSpeech.LEFT_HAND_ALREADY_CLOSED or
          speech_resp == RobotSpeech.RIGHT_ARM_ALREADY_HOLDING or
          speech_resp == RobotSpeech.RIGHT_ARM_ALREADY_RELEASED or
          speech_resp == RobotSpeech.LEFT_ARM_ALREADY_HOLDING or
          speech_resp == RobotSpeech.LEFT_ARM_ALREADY_RELEASED):
        Response.play_sound(RobotSound.ERROR)
    else:
        Response.play_sound(RobotSound.OTHER)

  @staticmethod
  def play_sound(requested_sound):
    ''' Plays the requested sound'''
    if (requested_sound in Response.all_sounds):
      Response._sound_client.playWave(os.path.join(Response._sounds_dir,
                                            requested_sound + '.wav'))
    else:
      Response._sound_client.playWave(os.path.join(Response._sounds_dir, 
                                                       'OTHER.wav'))

